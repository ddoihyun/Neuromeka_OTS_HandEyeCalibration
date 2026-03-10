# src/utils/logger.py
"""
통합 로거 모듈.

모든 모듈(단독 실행 / main.py 실행 공통)에서 아래처럼 사용한다:

    from src.utils.logger import get_logger
    log = get_logger(__name__)

    log.info("메시지")
    log.warning("경고")
    log.error("오류")
    log.debug("디버그")
    log.success("성공")       ← 커스텀 레벨 (INFO=20 < SUCCESS=25 < WARNING=30)
    log.section("섹션 제목")  ← 구분선 + 제목 출력

config.json 이 없는 환경(단독 실행 등)에서는 기본값(level=INFO, emoji=true)으로 동작한다.

config.json 예시:
    {
      "logging": {
        "level": "INFO",
        "emoji": true
      }
    }
"""

import logging
import sys
import json
from pathlib import Path
from typing import Optional

# ── ANSI 색상 코드 ────────────────────────────────────────────────────────────
_RESET = "\033[0m"
_BOLD  = "\033[1m"
_DIM   = "\033[2m"

_COL = {
    "DEBUG":    "\033[36m",      # cyan
    "INFO":     "\033[37m",      # white
    "SUCCESS":  "\033[92m",      # bright green
    "WARNING":  "\033[93m",      # bright yellow
    "ERROR":    "\033[91m",      # bright red
    "CRITICAL": "\033[41;97m",   # red bg + white text
}

# ── 이모티콘 ──────────────────────────────────────────────────────────────────
_EMOJI = {
    "DEBUG":    "🔍",
    "INFO":     "ℹ️ ",
    "SUCCESS":  "✅",
    "WARNING":  "⚠️ ",
    "ERROR":    "❌",
    "CRITICAL": "🚨",
}

# ── 커스텀 레벨 등록 ──────────────────────────────────────────────────────────
SUCCESS_LEVEL = 25   # INFO(20) < SUCCESS(25) < WARNING(30)
logging.addLevelName(SUCCESS_LEVEL, "SUCCESS")


# ── 컬러 포맷터 ───────────────────────────────────────────────────────────────
class _ColorFormatter(logging.Formatter):
    def __init__(self, use_emoji: bool = True, use_color: bool = True):
        super().__init__()
        self.use_emoji = use_emoji
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        level = record.levelname
        col   = _COL.get(level, "") if self.use_color else ""
        emoji = (_EMOJI.get(level, "  ") + " ") if self.use_emoji else ""
        reset = _RESET if self.use_color else ""
        bold  = _BOLD  if self.use_color else ""
        dim   = _DIM   if self.use_color else ""

        time_str   = self.formatTime(record, "%H:%M:%S")
        name_short = record.name.split(".")[-1]   # 마지막 패키지명만 표시

        level_tag = f"{col}{bold}[{level:<8}]{reset}"
        header    = f"{dim}{time_str}{reset} {level_tag} {dim}({name_short}){reset}"
        message   = f"{col}{record.getMessage()}{reset}"

        if record.exc_info:
            message += "\n" + self.formatException(record.exc_info)

        return f"{header}  {emoji}{message}"


# ── 커스텀 Logger 클래스 ──────────────────────────────────────────────────────
class _AppLogger(logging.Logger):
    """success() / section() 편의 메서드를 추가한 Logger."""

    def success(self, msg, *args, **kwargs):
        """INFO보다 한 단계 높은 성공 메시지 (초록색)."""
        if self.isEnabledFor(SUCCESS_LEVEL):
            self._log(SUCCESS_LEVEL, msg, args, **kwargs)

    def section(self, title: str, width: int = 60):
        """구분선 + 제목을 INFO 레벨로 출력."""
        sep = "─" * width
        self.info(f"\n{sep}\n  {title}\n{sep}")


logging.setLoggerClass(_AppLogger)

# ── 전역 설정 캐시 ────────────────────────────────────────────────────────────
_configured = False
_use_emoji  = True


def _find_config() -> Optional[dict]:
    """config.json 을 현재 파일 기준으로 상위 4단계까지 탐색."""
    here = Path(__file__).resolve()
    search_paths = [here.parent] + [here.parents[i] for i in range(1,5)]
    for parent in search_paths:
        candidate = parent / "config.json"
        if candidate.exists():
            try:
                with open(candidate, encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                return None
    return None


def configure_logging(level: Optional[str] = None, emoji: Optional[bool] = None):
    """
    루트 로거를 한 번만 설정한다. 이후 호출은 무시된다.

    Parameters
    ----------
    level : str  – "DEBUG" | "INFO" | "WARNING" | "ERROR"
                   (config.json 의 logging.level 이 우선)
    emoji : bool – 이모티콘 사용 여부
                   (config.json 의 logging.emoji 가 우선)
    """
    global _configured, _use_emoji

    if _configured:
        return

    cfg_level = (level or "INFO").upper()
    cfg_emoji = emoji if emoji is not None else True

    config = _find_config()
    if config:
        log_cfg   = config.get("logging", {})
        cfg_level = log_cfg.get("level", cfg_level).upper()
        cfg_emoji = bool(log_cfg.get("emoji", cfg_emoji))

    _use_emoji = cfg_emoji

    # Windows 콘솔 ANSI 색상 활성화
    use_color = True
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
        except Exception:
            use_color = False

    numeric_level = getattr(logging, cfg_level, logging.INFO)

    root = logging.getLogger()
    root.handlers.clear()

    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_ColorFormatter(use_emoji=cfg_emoji, use_color=use_color))
    root.addHandler(handler)
    root.setLevel(numeric_level)

    _configured = True


def get_logger(name: str) -> "_AppLogger":
    """
    통합 로거 반환. 최초 호출 시 자동으로 configure_logging()을 실행한다.

    Parameters
    ----------
    name : str – 보통 __name__ 을 그대로 전달

    Returns
    -------
    _AppLogger – info / warning / error / debug / success / section 메서드 제공

    Examples
    --------
    >>> from src.utils.logger import get_logger
    >>> log = get_logger(__name__)
    >>> log.section("1단계: 데이터 로딩")
    >>> log.info("CSV 파일 로드 중...")
    >>> log.success("로드 완료 – 120 rows")
    >>> log.warning("샘플 부족: 3/10")
    >>> log.error("파일을 찾을 수 없습니다")
    """
    configure_logging()
    return logging.getLogger(name)  # type: ignore[return-value]