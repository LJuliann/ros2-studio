//! Deterministic initial placement, not a continuously running force simulation.
//! Collapse cycles to assign dependency columns, separate disconnected families,
//! and sweep branch ordering to reduce crossings between adjacent columns.

use std::collections::{BTreeMap, BTreeSet};

use ros_studio_model::Project;

use crate::{GraphEdge, NODE_HEIGHT, NODE_WIDTH, NodePosition};

const COLUMN_SPACING: f32 = NODE_WIDTH + 260.0;
const ROW_SPACING: f32 = NODE_HEIGHT + 80.0;
const FAMILY_GAP: f32 = 80.0;
const ORDERING_SWEEPS: usize = 8;

struct LayoutGraph<'a> {
    ids: Vec<&'a str>,
    outgoing: Vec<Vec<usize>>,
    incoming: Vec<Vec<usize>>,
}

impl<'a> LayoutGraph<'a> {
    fn new(project: &'a Project, edges: &[GraphEdge]) -> Self {
        let mut nodes = project.nodes.iter().collect::<Vec<_>>();
        nodes.sort_by_key(|node| (&node.logical_name, &node.id));
        let ids = nodes
            .iter()
            .map(|node| node.id.as_str())
            .collect::<Vec<_>>();
        let indices = ids
            .iter()
            .enumerate()
            .map(|(index, &id)| (id, index))
            .collect::<BTreeMap<_, _>>();
        // Parallel topics must not bias ranks or branch ordering. Ignore self-links
        // and incomplete edges: neither introduces a dependency between two nodes.
        let connections = edges
            .iter()
            .filter_map(|edge| {
                let source = *indices.get(edge.publisher_id.as_str())?;
                let target = *indices.get(edge.subscriber_id.as_str())?;
                (source != target).then_some((source, target))
            })
            .collect::<BTreeSet<_>>();
        let mut outgoing = vec![Vec::new(); ids.len()];
        let mut incoming = vec![Vec::new(); ids.len()];
        for (source, target) in connections {
            outgoing[source].push(target);
            incoming[target].push(source);
        }
        Self {
            ids,
            outgoing,
            incoming,
        }
    }

    fn families(&self) -> Vec<Vec<usize>> {
        let mut visited = vec![false; self.ids.len()];
        let mut families = Vec::new();
        for start in 0..self.ids.len() {
            if visited[start] {
                continue;
            }
            let mut family = Vec::new();
            let mut pending = vec![start];
            visited[start] = true;
            while let Some(node) = pending.pop() {
                family.push(node);
                for &neighbor in self.outgoing[node].iter().chain(&self.incoming[node]) {
                    if !visited[neighbor] {
                        visited[neighbor] = true;
                        pending.push(neighbor);
                    }
                }
            }
            family.sort_unstable();
            families.push(family);
        }
        // Put connected systems first, then isolated nodes; ties stay deterministic.
        families.sort_by_key(|family| (std::cmp::Reverse(family.len()), family[0]));
        families
    }

    fn dependency_columns(&self) -> Vec<usize> {
        // Iterative Kosaraju avoids recursion on long chains. Every strongly
        // connected component (a feedback loop) occupies one dependency column.
        let mut visited = vec![false; self.ids.len()];
        let mut finished = Vec::new();
        for start in 0..self.ids.len() {
            if visited[start] {
                continue;
            }
            visited[start] = true;
            let mut stack = vec![(start, 0)];
            while let Some((node, next)) = stack.last_mut() {
                if let Some(&neighbor) = self.outgoing[*node].get(*next) {
                    *next += 1;
                    if !visited[neighbor] {
                        visited[neighbor] = true;
                        stack.push((neighbor, 0));
                    }
                } else {
                    finished.push(*node);
                    stack.pop();
                }
            }
        }
        let mut components = vec![usize::MAX; self.ids.len()];
        let mut component_count = 0;
        for start in finished.into_iter().rev() {
            if components[start] != usize::MAX {
                continue;
            }
            components[start] = component_count;
            let mut pending = vec![start];
            while let Some(node) = pending.pop() {
                for &neighbor in &self.incoming[node] {
                    if components[neighbor] == usize::MAX {
                        components[neighbor] = component_count;
                        pending.push(neighbor);
                    }
                }
            }
            component_count += 1;
        }
        let mut dependencies = vec![BTreeSet::new(); component_count];
        let mut incoming = vec![0; component_count];
        for (source, neighbors) in self.outgoing.iter().enumerate() {
            for &target in neighbors {
                let (source, target) = (components[source], components[target]);
                if source != target && dependencies[source].insert(target) {
                    incoming[target] += 1;
                }
            }
        }
        let mut ready = incoming
            .iter()
            .enumerate()
            .filter_map(|(index, &count)| (count == 0).then_some(index))
            .collect::<BTreeSet<_>>();
        let mut columns = vec![0; component_count];
        while let Some(source) = ready.pop_first() {
            for &target in &dependencies[source] {
                columns[target] = columns[target].max(columns[source] + 1);
                incoming[target] -= 1;
                if incoming[target] == 0 {
                    ready.insert(target);
                }
            }
        }
        components
            .into_iter()
            .map(|component| columns[component])
            .collect()
    }
}

fn centered_rows(layers: &[Vec<usize>], node_count: usize) -> Vec<f64> {
    let height = layers.iter().map(Vec::len).max().unwrap_or_default();
    let mut rows = vec![0.0; node_count];
    for layer in layers {
        let padding = (height - layer.len()) as f64 / 2.0;
        for (row, &node) in layer.iter().enumerate() {
            rows[node] = row as f64 + padding;
        }
    }
    rows
}

fn ordering_score(
    graph: &LayoutGraph<'_>,
    layers: &[Vec<usize>],
    columns: &[usize],
) -> (usize, f64) {
    let rows = centered_rows(layers, graph.ids.len());
    let mut target_rows = vec![0; graph.ids.len()];
    let mut crossings = 0;
    let mut length = 0.0;
    for (column, layer) in layers.iter().enumerate() {
        let Some(next_layer) = layers.get(column + 1) else {
            continue;
        };
        for (row, &node) in next_layer.iter().enumerate() {
            target_rows[node] = row;
        }
        let mut connections = Vec::new();
        for (source_row, &source) in layer.iter().enumerate() {
            for &target in &graph.outgoing[source] {
                if columns[target] > column {
                    length += (rows[source] - rows[target]).powi(2);
                }
                if columns[target] == column + 1 {
                    connections.push((source_row, target_rows[target]));
                }
            }
        }
        connections.sort_unstable();
        // A crossing is a strict inversion of target rows. A prefix-count tree
        // counts these in O(E log V), without pairwise comparisons on large graphs.
        // Sorting equal-source links by target also excludes shared-endpoint links.
        let mut prefix_counts = vec![0; next_layer.len() + 1];
        for (seen, (_, target_row)) in connections.into_iter().enumerate() {
            let mut index = target_row + 1;
            let mut not_above = 0;
            while index > 0 {
                not_above += prefix_counts[index];
                index &= index - 1;
            }
            crossings += seen - not_above;
            index = target_row + 1;
            while index < prefix_counts.len() {
                prefix_counts[index] += 1;
                index += index.isolate_lowest_one();
            }
        }
    }
    (crossings, length)
}

fn order_branches(graph: &LayoutGraph<'_>, layers: &mut Vec<Vec<usize>>, columns: &[usize]) {
    if layers.iter().all(|layer| layer.len() <= 1) {
        return;
    }
    let height = layers.iter().map(Vec::len).max().unwrap_or_default();
    let mut best = layers.clone();
    let mut best_score = ordering_score(graph, layers, columns);
    let mut rows = centered_rows(layers, graph.ids.len());
    for _ in 0..ORDERING_SWEEPS {
        for forward in [true, false] {
            for step in 0..layers.len() {
                let column = if forward {
                    step
                } else {
                    layers.len() - 1 - step
                };
                let mut scores = BTreeMap::new();
                for &node in &layers[column] {
                    let neighbors = if forward {
                        &graph.incoming[node]
                    } else {
                        &graph.outgoing[node]
                    };
                    let mut total = 0.0;
                    let mut count = 0;
                    for &neighbor in neighbors {
                        if (forward && columns[neighbor] < column)
                            || (!forward && columns[neighbor] > column)
                        {
                            total += rows[neighbor];
                            count += 1;
                        }
                    }
                    scores.insert(
                        node,
                        if count == 0 {
                            rows[node]
                        } else {
                            total / f64::from(count)
                        },
                    );
                }
                layers[column].sort_by(|left, right| {
                    scores[left]
                        .total_cmp(&scores[right])
                        .then_with(|| rows[*left].total_cmp(&rows[*right]))
                });
                let padding = (height - layers[column].len()) as f64 / 2.0;
                for (row, &node) in layers[column].iter().enumerate() {
                    rows[node] = row as f64 + padding;
                }
            }
            let score = ordering_score(graph, layers, columns);
            if score < best_score {
                best_score = score;
                best = layers.clone();
            }
        }
    }
    *layers = best;
}

/// A single fork with unequal paths into a terminal join reads more compactly
/// as a main chain above an alternate path, with their join below the chain end.
/// Only apply this to that simple DAG shape; keep the general layered layout for
/// symmetric diamonds, multiple forks/joins, feedback loops, or multiple sinks.
fn compact_terminal_join(
    graph: &LayoutGraph<'_>,
    family: &[usize],
    columns: &[usize],
    top: f32,
) -> Option<Vec<NodePosition>> {
    let joins = family
        .iter()
        .copied()
        .filter(|&node| graph.incoming[node].len() > 1)
        .collect::<Vec<_>>();
    let [join] = joins.as_slice() else {
        return None;
    };
    let join = *join;
    if graph.incoming[join].len() != 2
        || !graph.outgoing[join].is_empty()
        || columns[join] < 3
        || columns[graph.incoming[join][0]] == columns[graph.incoming[join][1]]
        || family
            .iter()
            .filter(|&&node| graph.outgoing[node].len() > 1)
            .count()
            != 1
        || family
            .iter()
            .filter(|&&node| graph.incoming[node].is_empty())
            .count()
            != 1
        || family
            .iter()
            .filter(|&&node| graph.outgoing[node].is_empty())
            .count()
            != 1
        || family.iter().any(|&node| {
            graph.outgoing[node].len() > 2
                || graph.outgoing[node]
                    .iter()
                    .any(|&target| columns[target] <= columns[node])
        })
    {
        return None;
    }
    // Follow the longest dependency path back from the join. Names/IDs break
    // ties deterministically because LayoutGraph's indices are sorted by them.
    let mut main_path = Vec::new();
    let mut current = join;
    while let Some(&parent) = graph.incoming[current]
        .iter()
        .max_by_key(|&&parent| (columns[parent], std::cmp::Reverse(parent)))
    {
        main_path.push(parent);
        current = parent;
    }
    main_path.reverse();
    let main_nodes = main_path.iter().copied().collect::<BTreeSet<_>>();
    let mut alternate = family
        .iter()
        .copied()
        .filter(|&node| node != join && !main_nodes.contains(&node))
        .collect::<Vec<_>>();
    alternate.sort_by_key(|&node| (columns[node], node));
    if alternate.is_empty() {
        return None;
    }
    let end_column = columns[*main_path.last()?] as f32;
    let mut positions = main_path
        .iter()
        .map(|&node| NodePosition {
            node_id: graph.ids[node].to_owned(),
            x: 48.0 + columns[node] as f32 * COLUMN_SPACING,
            y: top,
        })
        .collect::<Vec<_>>();
    for node in alternate {
        // Start the alternate branch below its fork, then progress to the right.
        positions.push(NodePosition {
            node_id: graph.ids[node].to_owned(),
            x: 48.0 + columns[node].checked_sub(1)? as f32 * COLUMN_SPACING,
            y: top + ROW_SPACING,
        });
    }
    positions.push(NodePosition {
        node_id: graph.ids[join].to_owned(),
        x: 48.0 + (end_column - 0.45) * COLUMN_SPACING,
        y: top + ROW_SPACING,
    });
    Some(positions)
}

pub(crate) fn graph_positions(project: &Project, edges: &[GraphEdge]) -> Vec<NodePosition> {
    let graph = LayoutGraph::new(project, edges);
    let columns = graph.dependency_columns();
    let mut positions = Vec::with_capacity(graph.ids.len());
    let mut top = 48.0;
    for family in graph.families() {
        if let Some(compact) = compact_terminal_join(&graph, &family, &columns, top) {
            positions.extend(compact);
            top += ROW_SPACING + NODE_HEIGHT + FAMILY_GAP;
            continue;
        }
        let last_column = family
            .iter()
            .map(|&node| columns[node])
            .max()
            .unwrap_or_default();
        let mut layers = vec![Vec::new(); last_column + 1];
        for node in family {
            layers[columns[node]].push(node);
        }
        order_branches(&graph, &mut layers, &columns);
        let height = layers.iter().map(Vec::len).max().unwrap_or_default();
        for (column, layer) in layers.iter().enumerate() {
            let padding = (height - layer.len()) as f32 / 2.0;
            for (row, &node) in layer.iter().enumerate() {
                positions.push(NodePosition {
                    node_id: graph.ids[node].to_owned(),
                    x: 48.0 + column as f32 * COLUMN_SPACING,
                    y: top + (row as f32 + padding) * ROW_SPACING,
                });
            }
        }
        top += (height.saturating_sub(1)) as f32 * ROW_SPACING + NODE_HEIGHT + FAMILY_GAP;
    }
    positions
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::{Context as _, Result};
    use ros_studio_model::{EntityId, Node, RuntimeState};

    fn project(names: &[&str]) -> Project {
        Project {
            id: EntityId::new("test"),
            name: "test".into(),
            root_path: "/tmp/test".into(),
            packages: vec![],
            nodes: names
                .iter()
                .map(|&name| Node {
                    id: EntityId::new(name),
                    logical_name: name.into(),
                    package_id: EntityId::new("package"),
                    executable: name.into(),
                    source_locations: vec![],
                    endpoints: vec![],
                    evidence: vec![],
                    runtime_state: RuntimeState::Unknown,
                })
                .collect(),
        }
    }

    fn edges(connections: &[(&str, &str)]) -> Vec<GraphEdge> {
        connections
            .iter()
            .map(|&(source, target)| GraphEdge {
                publisher_id: source.into(),
                subscriber_id: target.into(),
                topic: "/test".into(),
                type_name: "test/Message".into(),
            })
            .collect()
    }

    fn position<'a>(positions: &'a [NodePosition], id: &str) -> Result<&'a NodePosition> {
        positions
            .iter()
            .find(|position| position.node_id == id)
            .context("Missing position")
    }

    fn no_overlap(positions: &[NodePosition]) {
        for (index, left) in positions.iter().enumerate() {
            for right in &positions[index + 1..] {
                assert!(
                    (left.x - right.x).abs() >= NODE_WIDTH + 24.0
                        || (left.y - right.y).abs() >= NODE_HEIGHT + 24.0
                );
            }
        }
    }

    #[test]
    fn chains_flow_left_to_right_on_a_single_row() -> Result<()> {
        let positions = graph_positions(
            &project(&["camera", "detector", "navigation", "bridge"]),
            &edges(&[
                ("camera", "detector"),
                ("detector", "navigation"),
                ("navigation", "bridge"),
            ]),
        );
        for pair in ["camera", "detector", "navigation", "bridge"].windows(2) {
            let (left, right) = (
                position(&positions, pair[0])?,
                position(&positions, pair[1])?,
            );
            assert!(left.x < right.x);
            assert_eq!(left.y, right.y);
        }
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn unequal_paths_use_a_main_row_and_a_compact_secondary_branch() -> Result<()> {
        let project = project(&[
            "camera",
            "detector",
            "navigation",
            "test",
            "bridge",
            "isolated",
        ]);
        let connections = edges(&[
            ("camera", "detector"),
            ("detector", "navigation"),
            ("navigation", "bridge"),
            ("camera", "test"),
            ("test", "bridge"),
        ]);
        let positions = graph_positions(&project, &connections);
        let camera = position(&positions, "camera")?;
        let detector = position(&positions, "detector")?;
        let navigation = position(&positions, "navigation")?;
        let branch = position(&positions, "test")?;
        let join = position(&positions, "bridge")?;
        assert_eq!(camera.y, detector.y);
        assert_eq!(detector.y, navigation.y);
        assert!(camera.x < detector.x && detector.x < navigation.x);
        assert_eq!(branch.x, camera.x);
        assert_eq!(branch.y, join.y);
        assert_eq!(branch.y - camera.y, ROW_SPACING);
        assert!(detector.x < join.x && join.x < navigation.x);
        assert!(position(&positions, "isolated")?.y >= join.y + NODE_HEIGHT + FAMILY_GAP);
        no_overlap(&positions);
        let mut reordered = project;
        reordered.nodes.reverse();
        let mut reordered_edges = connections;
        reordered_edges.reverse();
        assert_eq!(graph_positions(&reordered, &reordered_edges), positions);
        Ok(())
    }

    #[test]
    fn compact_join_handles_a_branch_after_a_shared_prefix() -> Result<()> {
        let positions = graph_positions(
            &project(&["source", "fork", "main", "branch", "consumer", "join"]),
            &edges(&[
                ("source", "fork"),
                ("fork", "main"),
                ("main", "consumer"),
                ("consumer", "join"),
                ("fork", "branch"),
                ("branch", "join"),
            ]),
        );
        assert_eq!(
            position(&positions, "branch")?.x,
            position(&positions, "fork")?.x
        );
        assert_eq!(
            position(&positions, "source")?.y,
            position(&positions, "consumer")?.y
        );
        assert_eq!(
            position(&positions, "branch")?.y,
            position(&positions, "join")?.y
        );
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn telemetry_forks_keep_their_layered_layout_with_tighter_rows() -> Result<()> {
        let positions = graph_positions(
            &project(&["telemetry", "command", "bridge"]),
            &edges(&[("telemetry", "command"), ("telemetry", "bridge")]),
        );
        let (telemetry, command, bridge) = (
            position(&positions, "telemetry")?,
            position(&positions, "command")?,
            position(&positions, "bridge")?,
        );
        assert_eq!(command.x, bridge.x);
        assert!(telemetry.x < command.x);
        assert_eq!((command.y - bridge.y).abs(), NODE_HEIGHT + 80.0);
        assert_eq!(telemetry.y, (command.y + bridge.y) / 2.0);
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn connected_families_and_isolated_nodes_are_separated() -> Result<()> {
        let positions = graph_positions(
            &project(&["a", "b", "c", "d", "isolated"]),
            &edges(&[("a", "b"), ("c", "d")]),
        );
        assert_eq!(position(&positions, "a")?.y, position(&positions, "b")?.y);
        assert_eq!(position(&positions, "c")?.y, position(&positions, "d")?.y);
        assert!(
            position(&positions, "c")?.y >= position(&positions, "a")?.y + NODE_HEIGHT + FAMILY_GAP
        );
        assert!(position(&positions, "isolated")?.y > position(&positions, "d")?.y);
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn branch_ordering_removes_an_alphabetical_crossing() -> Result<()> {
        let project = project(&["root", "a", "b", "x", "y"]);
        let edges = edges(&[("root", "a"), ("root", "b"), ("a", "y"), ("b", "x")]);
        let graph = LayoutGraph::new(&project, &edges);
        let columns = graph.dependency_columns();
        let mut layers = vec![Vec::new(); 3];
        for (node, &column) in columns.iter().enumerate() {
            layers[column].push(node);
        }
        assert_eq!(ordering_score(&graph, &layers, &columns).0, 1);
        order_branches(&graph, &mut layers, &columns);
        assert_eq!(ordering_score(&graph, &layers, &columns).0, 0);
        let positions = graph_positions(&project, &edges);
        assert!(
            (position(&positions, "a")?.y - position(&positions, "b")?.y)
                * (position(&positions, "y")?.y - position(&positions, "x")?.y)
                > 0.0
        );
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn feedback_loops_do_not_hide_downstream_dependencies() -> Result<()> {
        let positions = graph_positions(
            &project(&["a", "b", "consumer", "sink"]),
            &edges(&[
                ("a", "b"),
                ("b", "a"),
                ("a", "a"),
                ("b", "consumer"),
                ("consumer", "sink"),
            ]),
        );
        assert_eq!(position(&positions, "a")?.x, position(&positions, "b")?.x);
        assert!(position(&positions, "b")?.x < position(&positions, "consumer")?.x);
        assert!(position(&positions, "consumer")?.x < position(&positions, "sink")?.x);
        no_overlap(&positions);
        Ok(())
    }

    #[test]
    fn placement_is_independent_of_scan_order_and_parallel_topics() {
        let mut project = project(&["root", "a", "b", "x", "y", "isolated"]);
        let mut edges = edges(&[
            ("root", "a"),
            ("root", "b"),
            ("a", "y"),
            ("b", "x"),
            ("missing", "root"),
        ]);
        let expected = graph_positions(&project, &edges);
        project.nodes.reverse();
        edges.reverse();
        edges.push(GraphEdge {
            publisher_id: "root".into(),
            subscriber_id: "a".into(),
            topic: "/another_topic".into(),
            type_name: "test/Message".into(),
        });
        assert_eq!(graph_positions(&project, &edges), expected);
    }

    #[test]
    fn empty_projects_and_shared_endpoint_links_have_no_crossings() {
        assert!(graph_positions(&project(&[]), &[]).is_empty());
        let project = project(&["root", "a", "b"]);
        let graph = LayoutGraph::new(&project, &edges(&[("root", "a"), ("root", "b")]));
        let columns = graph.dependency_columns();
        let layers = vec![vec![2], vec![0, 1]];
        assert_eq!(ordering_score(&graph, &layers, &columns).0, 0);
        no_overlap(&graph_positions(
            &project,
            &edges(&[("root", "a"), ("root", "b")]),
        ));
    }

    #[test]
    fn long_chains_do_not_require_recursive_traversal_or_ordering_sweeps() {
        let names = (0..2000)
            .map(|index| format!("node_{index:04}"))
            .collect::<Vec<_>>();
        let project = project(&names.iter().map(String::as_str).collect::<Vec<_>>());
        let connections = names
            .windows(2)
            .map(|pair| (pair[0].as_str(), pair[1].as_str()))
            .collect::<Vec<_>>();
        let positions = graph_positions(&project, &edges(&connections));
        assert_eq!(positions.len(), 2000);
        assert!(
            positions
                .windows(2)
                .all(|pair| pair[0].x < pair[1].x && pair[0].y == pair[1].y)
        );
    }
}
