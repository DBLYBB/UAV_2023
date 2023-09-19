import logging
import os
import sys
import colorlog  # 控制台日志输入颜色


def get_logger():
    #配置颜色
    log_colors_config = {
        "DEBUG": "cyan",
        "INFO": "bold_green",
        "WARNING": "bold_yellow",
        "ERROR": "bold_red",
        "CRITICAL": "bold_purple",
    }
    text_colors_config = {
        "DEBUG": "cyan",
        "INFO": "white",
        "WARNING": "bold_yellow",
        "ERROR": "bold_red",
        "CRITICAL": "bold_purple",
    }
    logger = logging.getLogger()
    #生成日志文件
    path = os.path.dirname(os.path.realpath(sys.argv[0]))
    logFile = os.path.join(path, "fc_log.log")
    logger.setLevel(logging.DEBUG)

    fmtConsole = colorlog.ColoredFormatter(
        "\n\n%(log_color)s[%(levelname)s] %(asctime)s %(message_log_color)s%(message)s",
        "%Y-%m-%d %H:%M:%S",
        log_colors=log_colors_config,
        secondary_log_colors={"message": text_colors_config},
    )
    fmtFile = logging.Formatter(
        "[%(levelname)s] %(asctime)s\n%(message)s",
        "(%Y-%m-%d %H:%M:%S)",
    )
    logger.fileHdl = logging.FileHandler(logFile, encoding="utf-8")
    logger.consoleHdl = logging.StreamHandler()
    logger.fileHdl.setLevel(logging.DEBUG)
    logger.consoleHdl.setLevel(logging.INFO)
    logger.fileHdl.setFormatter(fmtFile)
    logger.consoleHdl.setFormatter(fmtConsole)
    logger.addHandler(logger.fileHdl)
    logger.addHandler(logger.consoleHdl)
    return logger


logger = get_logger()


def print(msg, *args, **kwargs):
    for a in args:
        msg += f" {a}"
    logger.info(msg)


def Exception_Catcher(func):
    def wrapper(*args, **kwargs):
        import traceback

        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.error(e)
            logger.debug(traceback.format_exc())
            raise e

    return wrapper
