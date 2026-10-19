"""Filesystem model inventory and explicit deployment-contract association."""
from pathlib import Path
from .catalog import Catalog, ConfigError, path_in, load_yaml
from .references import resolve_values

def contained(path, parent):
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


MODEL_SUFFIXES = {'.onnx', '.pt', '.pth', '.jit', '.torchscript', '.141fonnx'}


def model_inventory(root, catalog=None, details=None):
    root = Path(root).resolve()
    catalog = catalog or Catalog(root)
    policies = [p for p in catalog.list() if p['kind'] == 'policy']
    if details is None:
        details = {p['id']: resolve_values(load_yaml(root / p['manifest']), root)
                   for p in policies if p['deployment']['status'] == 'ready'}
    result = []
    for file in sorted((root / 'policies').glob('*/*/**/*')):
        if not file.is_file() or file.suffix.lower() not in MODEL_SUFFIXES:
            continue
        relative = file.relative_to(root).as_posix()
        try:
            target = path_in(root / 'policies', file)
        except ConfigError:
            continue
        candidates = [p for p in policies if contained(target, path_in(root, p['deployment']['model_directory'], exists=False))]
        if candidates:
            depth = max(len(Path(p['deployment']['model_directory']).parts) for p in candidates)
            candidates = [p for p in candidates if len(Path(p['deployment']['model_directory']).parts) == depth]
        exact = [p for p in candidates if details.get(p['id'], {}).get('model', {}).get('file') == relative]
        ready = [p for p in candidates if p['deployment']['status'] == 'ready']
        contract = (exact or ready or candidates or [None])[0]
        selectable = bool(ready) and file.suffix.lower() == '.onnx'
        if not contract:
            reason = '未登记部署接口'
        elif contract['deployment']['status'] != 'ready':
            reason = '；'.join(contract['deployment']['diagnostics']) or '缺少可用部署接口'
        elif file.suffix.lower() != '.onnx':
            reason = '当前推理后端仅支持 ONNX' if file.suffix.lower() != '.141fonnx' else '文件扩展名异常，未按 ONNX 加载'
        else:
            reason = '沿用同目录部署接口，输入输出与控制语义待验证' if not exact else '已关联部署接口，模型待校验'
        result.append({'file': relative, 'name': file.name, 'format': file.suffix.lower().lstrip('.'),
                       'robots': sorted({r for p in candidates for r in p['data']['robots']}),
                       'policy': contract['id'] if contract else None, 'selectable': selectable,
                       'registered': bool(exact), 'status': 'unverified' if selectable else 'unavailable',
                       'reason': reason})
    return result


def selected_model(root, robot, policy, filename, catalog=None):
    # Use the same ownership/interface association as the browser inventory.
    entry = next((m for m in model_inventory(root, catalog) if m['file'] == filename and robot in m['robots'] and m['policy'] == policy), None)
    if not entry or not entry['selectable']:
        raise ConfigError('Selected model does not have a supported deployment interface for this robot: ' + str(filename))
    return entry
