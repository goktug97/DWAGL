html:
	emcc main.c ./DynamicWindowApproach/src/dwa.c -o dwa.html -s USE_WEBGL2=1  -isystem./cglm/include -isystem./DynamicWindowApproach/src -O3

js:
	emcc main.c ./DynamicWindowApproach/src/dwa.c -o dwa.js -s USE_WEBGL2=1  -isystem./cglm/include -isystem./DynamicWindowApproach/src -O3
