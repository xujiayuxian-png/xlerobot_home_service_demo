"""Pinned, process-local runtime layout for the X1 GenieX experiment."""
from pathlib import Path


def runtime_settings(config, config_path, root):
    from tools.lib.npu_profile import MODEL_GENIEX
    if config.get('model_id') != MODEL_GENIEX:
        raise ValueError('GenieX model configuration does not match the selected model')
    paths = {}
    for key in ('python', 'runtime_root', 'library_dir', 'model_path', 'mmproj_path'):
        value = config.get(key)
        if not isinstance(value, str) or not Path(value).is_absolute() or not Path(value).exists():
            raise ValueError(f'GenieX {key} must be an existing absolute path')
        paths[key] = Path(value)
    rt = paths['runtime_root']/'usr/lib/aarch64-linux-gnu'
    loader = rt/'ld-linux-aarch64.so.1'
    library = paths['library_dir']
    for file in (loader, library/'libgeniex.so', library/'llama_cpp/libggml-htp-v73.so'):
        if not file.is_file():
            raise ValueError(f'missing GenieX runtime artifact: {file.name}')
    required = {str(Path(config_path).resolve()), str(paths['python'].resolve()),
                str(paths['model_path'].resolve()), str(paths['mmproj_path'].resolve())}
    for directory in (paths['runtime_root'], library.parent):
        required.update(str(p.resolve()) for p in directory.rglob('*')
                        if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc')
    command = [str(loader), '--library-path', f'{rt}:/usr/local/lib:/usr/lib:/lib/aarch64-linux-gnu',
               str(paths['python']), str(root/'services/npu/geniex_server.py'),
               '--config', str(config_path)]
    env = {'ADSP_LIBRARY_PATH': f'{library}/llama_cpp;/usr/lib/rfsa/adsp;/adsp',
           'GENIEX_LOG': 'warn'}
    return required, command, env
