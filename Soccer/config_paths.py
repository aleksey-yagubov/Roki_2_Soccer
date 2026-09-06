from pathlib import Path


def repo_root(current_work_directory=None):
    if current_work_directory:
        return Path(current_work_directory).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def repo_root_str(current_work_directory=None):
    return repo_root(current_work_directory).as_posix() + "/"


def repo_path(current_work_directory, relative_path):
    return repo_root(current_work_directory) / relative_path


def _init_params_candidates(root, relative_path):
    init_root = root / "Init_params"
    return [
        init_root / "local" / relative_path,
        init_root / "defaults" / relative_path,
        init_root / relative_path,
    ]


def init_param_read_path(current_work_directory, relative_path):
    root = repo_root(current_work_directory)
    for candidate in _init_params_candidates(root, Path(relative_path)):
        if candidate.exists():
            return candidate
    return _init_params_candidates(root, Path(relative_path))[-1]


def init_param_write_path(current_work_directory, relative_path):
    root = repo_root(current_work_directory)
    path = root / "Init_params" / "local" / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    return path
