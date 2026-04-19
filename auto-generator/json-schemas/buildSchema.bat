@echo off
REM assume this is ran from R2026
npx ts-json-schema-generator --path auto-generator\types.ts --type Layout > auto-generator\json-schemas\layout.schema.json