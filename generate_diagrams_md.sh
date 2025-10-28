#!/bin/bash

echo -e "Checking for changes in UML files and regenerate diagrams if needed...\n"

# Check if plantuml exists
if ! command -v plantuml &> /dev/null; then
    echo "Error: PlantUML is not installed or not in PATH."
    exit 1
fi

# Check if diagram generation is needed
if ! make -C docs --question; then
  echo "Changes in docs detected. Generating PNG diagrams from PlantUML sources..."
  if ! make -C docs; then
    echo "ERROR: 'make' failed."
    exit 1
  else
    echo ""
  fi
  
else
  echo -e "Everything is up to date.\n"
fi


echo "Check and diagram generation completed."
