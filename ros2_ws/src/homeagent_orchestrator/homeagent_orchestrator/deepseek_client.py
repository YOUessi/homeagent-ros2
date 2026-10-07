import json
import urllib.error
import urllib.request
from typing import Dict

from .planner_core import SYSTEM_PROMPT, parse_plan_text


class DeepSeekError(RuntimeError):
    pass


class DeepSeekClient:
    def __init__(
        self,
        *,
        api_key: str,
        base_url: str = "https://api.deepseek.com",
        model: str = "deepseek-chat",
        timeout: float = 30.0,
    ) -> None:
        if not api_key:
            raise DeepSeekError("DEEPSEEK_API_KEY is empty")
        self.api_key = api_key
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def plan(self, command: str) -> Dict:
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": command},
            ],
            "temperature": 0,
            "stream": False,
        }
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise DeepSeekError(
                f"DeepSeek HTTP {exc.code}: {body[:300]}"
            ) from exc
        except urllib.error.URLError as exc:
            raise DeepSeekError(f"DeepSeek connection error: {exc}") from exc

        try:
            data = json.loads(raw)
            content = data["choices"][0]["message"]["content"]
        except (json.JSONDecodeError, KeyError, IndexError, TypeError) as exc:
            raise DeepSeekError("unexpected DeepSeek response schema") from exc

        return parse_plan_text(content)
