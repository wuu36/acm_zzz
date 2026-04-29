

```shell
uv init
uv sync
uv sync --group dev
uv sync --extra control
uv pip list
```

```shell
uv run pytest tests/ -v
uv run python examples/basic_simulation.py
```


```shell
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

```python
import pytest

@pytest.fixture
def motor_config()
...
```

```python
from matplotlib.axes import Axes
from collections.abc import Sequence
```