var defaultExtensionid = "0";
var Extensionid = defaultExtensionid;
var ExtnID = getQryStrParamValues('xid');
var validExtensionID = ["2"];
Extensionid = (validExtensionID.indexOf(ExtnID) != -1) ? ExtnID : defaultExtensionid;
var getMarket = getQryStrParamValues('bmkt');

function getQryStrParamValues(param) {
    var url = window.location.href.slice(window.location.href.indexOf('?') + 1).split('&');
    for (var i = 0; i < url.length; i++) {
        var urlparam = url[i].split('=');
        if (urlparam[0].toLowerCase() == param.toLowerCase()) {
            return urlparam[1];
        }
    }
}


