import cr_knowledge_extraction.cr_knowledge_extraction_core as core


def hello() -> str:
    return "Hello, CommonRoad (from Python)!"


def hello_cpp() -> str:
    return core.hello()
