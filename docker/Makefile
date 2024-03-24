all: help

help:
	@echo ""
	@echo "   Help Menu"
	@echo ""
	@echo "   make build                - builds images"
	@echo "   make run                  - runs container"
	@echo ""

build:
	@cd ../../; \
	docker build --rm -t tuw_geometry -f tuw_geometry/docker/Dockerfile .

run:
	@docker run -ti tuw_geometry
