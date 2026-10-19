"""Readable YAML: compact vectors and one record per observation/action/state."""
import yaml


class InlineMap(dict):
    pass


class CompactDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def sequence(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data,
                                     flow_style=all(not isinstance(v, (dict, list)) for v in data))


CompactDumper.add_representer(list, sequence)
CompactDumper.add_representer(InlineMap, lambda dumper, value: dumper.represent_mapping("tag:yaml.org,2002:map", value, flow_style=True))


def dumps(data):
    def compact(value, in_sequence=False):
        if isinstance(value, dict):
            flat = all(not isinstance(v, dict) and (not isinstance(v, list) or len(v) <= 4) for v in value.values())
            short = sum(len(str(k)) + len(str(v)) + 4 for k, v in value.items()) <= 130
            cls = InlineMap if (in_sequence or short) and flat else dict
            return cls((k, compact(v)) for k, v in value.items())
        if isinstance(value, list):
            return [compact(v, True) for v in value]
        return value
    return yaml.dump(compact(data), Dumper=CompactDumper, sort_keys=False, allow_unicode=True, width=140)
