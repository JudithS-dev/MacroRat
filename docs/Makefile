PUML_DIR=diagrams
IMG_DIR=img

PUMLS=$(wildcard $(PUML_DIR)/*.puml)
PNGS=$(patsubst $(PUML_DIR)/%.puml,$(IMG_DIR)/%.png,$(PUMLS))

all: $(PNGS)

$(IMG_DIR)/%.png: $(PUML_DIR)/%.puml
	@mkdir -p $(IMG_DIR)
	plantuml -tpng $< -o ../$(IMG_DIR)
