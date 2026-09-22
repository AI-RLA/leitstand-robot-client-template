PYTHON ?= python3

.PHONY: contract-check test lint

# leitstand.repos (the checkout) and setup.py (pip) both pin the contract, so check that they agree
# and that the installed one is that version.
contract-check:
	@repos=$$(sed -n '/leitstand-robot-contract:/,/version:/s/^ *version: *v//p' leitstand.repos); \
	pin=$$(grep -o 'leitstand-robot-contract==[0-9.]*' leitstand_client/setup.py | cut -d= -f3); \
	test "$$repos" = "$$pin" || { echo "leitstand.repos pins $$repos, setup.py pins $$pin"; exit 1; }; \
	$(PYTHON) -c "import importlib.metadata as m; \
	v = m.version('leitstand-robot-contract'); assert v == '$$repos', v; \
	from leitstand.robot.v1 import mission_pb2; \
	assert 'run_id' in [f.name for f in mission_pb2.Mission.DESCRIPTOR.fields]; \
	print('leitstand-robot-contract', v, 'ok')"

test:
	$(PYTHON) -m pytest -q

lint:
	$(PYTHON) -m ruff check . && $(PYTHON) -m ruff format --check .
