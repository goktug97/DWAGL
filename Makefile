html:
	emcc main.c ./DynamicWindowApproach/src/dwa.c -o dwa.html -s USE_WEBGL2=1  -isystem./cglm/include -isystem./DynamicWindowApproach/src -O3 -fsanitize=undefined -s EXPORTED_FUNCTIONS='["_update_config", "_main"]' -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]'

js:
	emcc main.c ./DynamicWindowApproach/src/dwa.c -o dwa.js -s USE_WEBGL2=1  -isystem./cglm/include -isystem./DynamicWindowApproach/src -O3
