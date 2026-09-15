PYTHON ?= python3

.PHONY: contract-check test lint

# The contract is installed from its own checkout, never from here; this only
# proves the installed one is the version this client speaks.
contract-check:
	$(PYTHON) -c "import importlib.metadata as m; \
	v = m.version('leitstand-robot-contract'); assert v == '0.4.0', v; \
	from leitstand.robot.v1 import mission_pb2; \
	assert 'run_id' in [f.name for f in mission_pb2.Mission.DESCRIPTOR.fields]; \
	print('leitstand-robot-contract', v, 'ok')"

test:
	$(PYTHON) -m pytest -q

lint:
	$(PYTHON) -m ruff check . && $(PYTHON) -m ruff format --check .
