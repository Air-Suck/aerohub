import json
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener


class WorkerError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class ModelClient:
    def __init__(self, endpoint, timeout):
        self.endpoint = endpoint.rstrip("/")
        self.timeout = timeout

    def request(self, route, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = Request(self.endpoint + route, data=data, headers={"Content-Type": "application/json"})
        try:
            # Model traffic is internal; never route it through workstation proxies.
            with build_opener(ProxyHandler({})).open(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as error:
            codes = {429: "BUSY", 422: "INVALID_INPUT", 503: "NOT_READY"}
            raise WorkerError(codes.get(error.code, "WORKER_ERROR"), error.read(4096).decode(errors="replace")) from error
        except (OSError, ValueError) as error:
            raise WorkerError("WORKER_UNAVAILABLE", str(error)) from error
