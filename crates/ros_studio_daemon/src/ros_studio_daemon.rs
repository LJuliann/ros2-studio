#![forbid(unsafe_code)]

use std::io::{self, BufRead, Write};

use ros_studio_protocol::{
    CAPABILITY_UNAVAILABLE, INVALID_PARAMS, INVALID_REQUEST, METHOD_NOT_FOUND, PARSE_ERROR,
    Request, RequestMessage, ResponseMessage, ResponseResult, UNSUPPORTED_PROTOCOL_VERSION,
};
use serde_json::Value;

pub mod runtime_graph;

#[cfg(feature = "ros-runtime")]
mod runtime;

#[cfg(feature = "ros-runtime")]
pub use runtime::serve_ros_stdio;

pub fn serve(reader: impl BufRead, mut writer: impl Write) -> io::Result<()> {
    for line in reader.lines() {
        let line = line?;
        let response = handle_line(&line);

        serde_json::to_writer(&mut writer, &response).map_err(io::Error::other)?;
        writer.write_all(b"\n")?;
        writer.flush()?;
    }

    Ok(())
}

fn handle_line(line: &str) -> ResponseMessage {
    match serde_json::from_str::<RequestMessage>(line) {
        Ok(request) => handle_request(request),
        Err(error) => malformed_request(line, &error),
    }
}

fn handle_request(message: RequestMessage) -> ResponseMessage {
    if let Err(error) = message.validate() {
        return ResponseMessage::error(
            Some(message.id),
            UNSUPPORTED_PROTOCOL_VERSION,
            error.to_string(),
        );
    }

    match message.request {
        Request::OpenWorkspace { path } if path.trim().is_empty() => {
            ResponseMessage::error(Some(message.id), INVALID_PARAMS, "workspace path is empty")
        }
        Request::OpenWorkspace { .. } | Request::StopRuntimeDiscovery {} => {
            ResponseMessage::success(message.id, ResponseResult::Accepted {})
        }
        Request::StartRuntimeDiscovery {}
        | Request::Launch { .. }
        | Request::GetParameters { .. }
        | Request::SetParameter { .. } => ResponseMessage::error(
            Some(message.id),
            CAPABILITY_UNAVAILABLE,
            "ROS runtime is not available in the dummy daemon",
        ),
    }
}

pub(crate) fn malformed_request(line: &str, error: &serde_json::Error) -> ResponseMessage {
    let value: Value = match serde_json::from_str(line) {
        Ok(value) => value,
        Err(_) => return ResponseMessage::error(None, PARSE_ERROR, error.to_string()),
    };

    let id = value.get("id").and_then(Value::as_u64);

    if value.get("jsonrpc").is_none() || value.get("protocol_version").is_none() {
        return ResponseMessage::error(id, INVALID_REQUEST, error.to_string());
    }

    let Some(method) = value.get("method").and_then(Value::as_str) else {
        return ResponseMessage::error(id, INVALID_REQUEST, error.to_string());
    };

    let known_method = matches!(
        method,
        "open_workspace"
            | "start_runtime_discovery"
            | "stop_runtime_discovery"
            | "launch"
            | "get_parameters"
            | "set_parameter"
    );

    let code = if known_method {
        INVALID_PARAMS
    } else {
        METHOD_NOT_FOUND
    };

    ResponseMessage::error(id, code, error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replies_to_multiple_requests_over_line_delimited_json() -> anyhow::Result<()> {
        let open = RequestMessage::new(
            1,
            Request::OpenWorkspace {
                path: "/workspace/drone".to_owned(),
            },
        );
        let start = RequestMessage::new(2, Request::StartRuntimeDiscovery {});
        let input = format!(
            "{}\n{}\n",
            serde_json::to_string(&open)?,
            serde_json::to_string(&start)?
        );

        let mut output = Vec::new();
        serve(input.as_bytes(), &mut output)?;

        let output = String::from_utf8(output)?;
        let responses = output
            .lines()
            .map(serde_json::from_str::<ResponseMessage>)
            .collect::<Result<Vec<_>, _>>()?;

        assert_eq!(responses.len(), 2);
        assert_eq!(responses[0].id, Some(1));
        assert_eq!(responses[0].result, Some(ResponseResult::Accepted {}));
        assert_eq!(responses[1].id, Some(2));
        assert_eq!(
            responses[1].error.as_ref().map(|error| error.code),
            Some(CAPABILITY_UNAVAILABLE)
        );

        Ok(())
    }

    #[test]
    fn rejects_unsupported_protocol_and_malformed_json() -> anyhow::Result<()> {
        let input = concat!(
            "{\"jsonrpc\":\"2.0\",\"protocol_version\":2,\"id\":7,\"method\":\"stop_runtime_discovery\",\"params\":{}}\n",
            "not json\n",
        );
        let mut output = Vec::new();
        serve(input.as_bytes(), &mut output)?;

        let output = String::from_utf8(output)?;
        let responses = output
            .lines()
            .map(serde_json::from_str::<ResponseMessage>)
            .collect::<Result<Vec<_>, _>>()?;

        assert_eq!(responses[0].id, Some(7));
        assert_eq!(
            responses[0].error.as_ref().map(|error| error.code),
            Some(UNSUPPORTED_PROTOCOL_VERSION)
        );
        assert_eq!(responses[1].id, None);
        assert_eq!(
            responses[1].error.as_ref().map(|error| error.code),
            Some(PARSE_ERROR)
        );

        Ok(())
    }

    #[test]
    fn rejects_unknown_methods_and_empty_workspace_paths() -> anyhow::Result<()> {
        let unknown =
            r#"{"jsonrpc":"2.0","protocol_version":1,"id":3,"method":"unknown","params":{}}"#;
        let empty_workspace = RequestMessage::new(
            4,
            Request::OpenWorkspace {
                path: " ".to_owned(),
            },
        );
        let input = format!("{unknown}\n{}\n", serde_json::to_string(&empty_workspace)?);
        let mut output = Vec::new();
        serve(input.as_bytes(), &mut output)?;

        let output = String::from_utf8(output)?;
        let responses = output
            .lines()
            .map(serde_json::from_str::<ResponseMessage>)
            .collect::<Result<Vec<_>, _>>()?;

        assert_eq!(responses[0].id, Some(3));
        assert_eq!(
            responses[0].error.as_ref().map(|error| error.code),
            Some(METHOD_NOT_FOUND)
        );
        assert_eq!(responses[1].id, Some(4));
        assert_eq!(
            responses[1].error.as_ref().map(|error| error.code),
            Some(INVALID_PARAMS)
        );

        Ok(())
    }
}
