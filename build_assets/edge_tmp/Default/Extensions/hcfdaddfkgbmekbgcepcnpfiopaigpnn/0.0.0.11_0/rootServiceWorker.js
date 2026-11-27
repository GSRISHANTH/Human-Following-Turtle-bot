
var serviceWorker = self;
var scriptsUsedByRootSW = [
    "background.js", "ping.js"
];

scriptsUsedByRootSW.forEach(function (script) {
    try {
        self.importScripts(script);
    }
    catch (e) {
        console.log(e);
    }
});
