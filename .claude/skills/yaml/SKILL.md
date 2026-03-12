# YAML & JSON Schema Conventions

Consult this when reading, writing, or modifying any YAML or JSON files in a project.

## Prefer Pydantic models for YAML I/O

Never use raw `yaml.safe_load()` / `yaml.dump()` directly in application code. Instead:

1. Define a Pydantic `BaseModel` that matches the YAML structure
2. Load via `model.model_validate(yaml.safe_load(path))` — gives type-safe attribute access and catches schema violations immediately
3. Save via `yaml.dump(model.model_dump(exclude_none=True), ...)` — produces clean YAML without `None` clutter

Wrap these in `load_*` / `save_*` helper functions so callers never touch raw dicts.

## yaml-language-server schema tags

When saving YAML, prepend a schema tag so VS Code (with the YAML extension) provides autocompletion and validation:

```yaml
# yaml-language-server: $schema=../../schemas/my_format.json
```

Use a relative path from the YAML file to the JSON Schema file. Save helpers should add this automatically.

Another approach is to publish schemas as GitHub Release artifacts and reference via absolute URL. (TODO: add details for ibek projects)

## JSON Schema generation from Pydantic

Generate JSON Schema files from Pydantic models using `model.model_json_schema()`:

```python
import json
schema = MyModel.model_json_schema()
Path("schemas/my_format.json").write_text(json.dumps(schema, indent=2) + "\n")
```

Commit generated schemas to the repo so VS Code picks them up without running code. Add a test that compares committed schemas against `model_json_schema()` output to catch staleness.

## Key Pydantic patterns for YAML

- Use `Literal["a", "b"]` for enum-like string fields — catches typos at validation time
- Use `model_config = ConfigDict(extra="allow")` only for formats with user-defined fields; keep all others strict
- Use `exclude_none=True` when dumping so optional fields don't clutter YAML with `null` entries
- For `bool | None` flags, use `None` as the default (not `False`) so absent flags are omitted from output
- Use Pydantic models as the data classes throughout — do not use `@dataclass` for structures that are serialized to/from YAML

## Adding a new YAML format

1. Define a Pydantic model
2. Add `load_*` / `save_*` helpers (with schema tag injection)
3. Generate and commit the JSON Schema
4. Add validation + round-trip tests
5. Add a staleness test comparing committed JSON against `model_json_schema()`
