import ast
from pathlib import Path


def test_proxy_server_imports_all_identity_helpers_before_runtime():
    project_root = Path(__file__).resolve().parents[2]
    source_path = project_root / "server/serve_proxy_piper_shadow.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    imported = {
        alias.name
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        and node.module == "server.policy_server.artifact_identity"
        for alias in node.names
    }
    assert {
        "files_sha256", "resolve_managed_tree", "sha256_file",
        "sha256_text_file", "tree_sha256",
    } <= imported
