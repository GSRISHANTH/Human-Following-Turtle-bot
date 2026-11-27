var strBrowserName = "me";
//global variable from default.js
var strExtnID = Extensionid;
var strExtnName = "";
var pageText1 = "";
var pageText2 = "";

//Market passed in query string
//var windowsMarketsAvailable = ['af-za', 'ar-sa', 'az-latn', 'bg-bg', 'bn-in', 'bs-latn', 'ca-es', 'cs-cz', 'da-dk', 'de-de', 'el-gr', 'en-ca', 'en-gb', 'en-us', 'es-419', 'es-es', 'et-ee', 'eu-es', 'eu-eu', 'fa-ir', 'fi-fi', 'fr-fr', 'ga-ie', 'gu-in', 'he-il', 'hr-hr', 'hu-hu', 'id-id', 'it-it', 'ja-jp', 'ka-ge', 'ko-kr', 'lt-lt', 'lv-lv', 'mk-mk', 'ms-my', 'mt-mt', 'nb-no', 'nl-nl', 'nn-no', 'pa-in', 'pl-pl', 'pt-br', 'pt-pt', 'ro-ro', 'ru-ru', 'sk-sk', 'sl-si', 'sq-al', 'sv-se', 'te-in', 'th-th', 'tr-tr', 'uk-ua', 'ur-pk', 'vi-vn', 'zh-cn', 'zh-hans', 'zh-hant', 'zh-tw'];
var windowsMarketsAvailable = ["af-za", "am-et", "ar-sa", "as-in", "az-latn-az", "bg-bg", "bn-in", "bs-latn-ba", "ca-es", "ca-es-valencia",
    "cs-cz", "cy-gb", "da-dk", "de-at", "de-ch", "de-de", "el-gr", "en-au", "en-ca", "en-gb", "en-in", "en-us", "es-ar", "es-es", 
    "es-mx", "et-ee", "eu-es", "fa-ir", "fi-fi", "fil-ph", "fr-be", "fr-ca", "fr-fr", "ga-ie", "gd-gb", "gl-es", "gu-in", "he-il", "hi-in", "hr-hr",
    "hu-hu", "id-id", "is-is", "it-it", "ja-jp", "ka-ge", "kk-kz", "km-kh", "kn-in", "ko-kr", "kok-in", "lb-lu", "lo-la", "lt-lt",
    "lv-lv", "mi-nz", "mk-mk", "ml-in", "mr-in", "ms-bn", "ms-my", "mt-mt", "nb-no", "ne-np", "nl-be", "nl-nl", "nn-no", "or-in", "pa-in",
    "pl-pl", "pt-br", "pt-pt", "quz-pe", "ro-ro", "ru-ru", "sk-sk", "sl-si", "sq-al", "sr-cyrl-ba", "sr-cyrl-rs",
    "sr-latn-cs", "sr-latn-rs", "sv-se", "ta-in", "te-in", "th-th", "tr-tr", "tt-ru", "ug-cn", "uk-ua", "ur-pk",
    "vi-vn", "zh-cn", "zh-hans", "zh-hant", "zh-hk", "zh-tw","es-419","sr-cyrl"];
//firefoxSupportedLanguages and correspondingWindowMarketForFirefoxSupportedLanguage arrays are related, So don't change the order(index)
//MF supported languages
var firefoxSupportedLanguages = ['en-us'];

//Corresponding markets for MF supported languages
var correspondingWindowMarketForFirefoxSupportedLanguage = ['en-us'];

//browser language
var userLanguage = navigator.language || navigator.userLanguage || "en-us";
userLanguage = userLanguage.toLowerCase();

//Selecting the language of which the page will be get loaded
var market = "en-us";

if (windowsMarketsAvailable.indexOf(getMarket) !== -1) {
    market = getMarket;
} else if (firefoxSupportedLanguages.indexOf(userLanguage) !== -1) {
    var language = correspondingWindowMarketForFirefoxSupportedLanguage[firefoxSupportedLanguages.indexOf(userLanguage)];
    market = language;
}
else {
    market = "en-us";
}

//Loading the page
window.onload = onload();

function onload() {
    init(market);
}

//To get the parameter's value
function getQryStrParamValues(param) {
    var url = window.location.href.slice(window.location.href.indexOf('?') + 1).split('&');
    for (var i = 0; i < url.length; i++) {
        var urlparam = url[i].split('=');
        if (urlparam[0].toLowerCase() == param.toLowerCase()) {
            if (urlparam[1] == undefined)
                return undefined;
            else
                return urlparam[1].toLowerCase();
        }
    }
}

function init(market) {
    document.getElementById("marketId").value = market;
    document.body.className = market;
    var rtlMarkets = ["ar-sa", "fa-ir", "he-il", "ur-pk", "ug-cn"];

    if (rtlMarkets.indexOf(market) != -1) {
        document.body.dir = "rtl";
        document.dir = "rtl";
        document.documentElement.lang = market;
        var isRtl = true;
    }

    try {
        $.getJSON('assets/json/ExtnName/' + strExtnID + '/messages.json', function (extnresponse) {
            console.log(market);
            
            if (extnresponse[market]) {
                strExtnName = extnresponse[market];
                
            }
            else {
                strExtnName = extnresponse["en-us"];
              
            }
        });
        //setTimeout(function () {
            fetch('assets/json/Common/' + market.toLowerCase() + '/messages.json').then(response => {
                return response.json();
            }).then(data => {
                // Parse JSON string into object
                var actual_JSON = data;
                // need to pass EXTENSION ID to json file to get mapping text
                document.title = actual_JSON.Heading;
                pageText1 = actual_JSON.Step2a;
                pageText2 = actual_JSON.Step1a;

                //To load banner, logo and prompt screenshot
                $(function () {
                    $.getJSON("scripts/extnDetails.json", function (responseJSON) {

                        var bannerImg = document.getElementById("image2");
                        bannerImgSource = responseJSON[strBrowserName][strExtnID]["banner"];
                        var logoImg = document.getElementById("logo");
                        logoImageSource = responseJSON[strBrowserName][strExtnID]["logo"];
                        document.getElementById("Step2a").innerHTML = pageText1;
                        
                        document.getElementById("Step1a").innerHTML = pageText2 + " " + strExtnName + ".";

                        if (bannerImgSource != "" && bannerImgSource != null && bannerImgSource != undefined) {
                            bannerImg.src = bannerImgSource;
                        }
                        else {
                            $("#image2").hide();
                        }

                        //Logo availability
                        if (logoImageSource != "" && logoImageSource != null && logoImageSource != undefined) {
                            logoImg.src = logoImageSource;
                        }
                        else {
                            $("#logo").hide();
                        }
                    });
                });
            }).catch(err => {
                // Do something for an error here
            });
    //    },200)

    }
    catch (err) {
    }
}

//Showing page for valid xid
if (Extensionid != 0) {
    document.body.style.opacity = 1;
}

////If user is updated directly to version 10 from any older versions - for inactive users
console.log("Set default values from json.js");
var date = new Date().toDateString();

////Set Migration flag
chrome.storage.local.set({ "Migration": "True" });

//pc
if (localStorage["pc"])
	chrome.storage.local.set({ "pc": localStorage["pc"] });
else
	chrome.storage.local.set({ "pc": "U474" });


//machineID 			
if (localStorage["MachineID"])
	chrome.storage.local.set({ "MachineID": localStorage["MachineID"] });
else
{
	chrome.storage.local.get(["MachineID"], (items) => {
        if (!items.MachineID) {
            chrome.storage.local.set({ "MachineID": guid() });
        }
    });													
}	

//channel  		   
if (localStorage["channel"])
	chrome.storage.local.set({ "channel": localStorage["channel"] });
else
	chrome.storage.local.set({ "channel": "organic" });


//pingdate 			   
if (localStorage["PingDate"])
	chrome.storage.local.set({ "PingDate": localStorage["PingDate"] });

//dpc		 
if (localStorage["_dpc"])
	chrome.storage.local.set({ "_dpc": localStorage["_dpc"] });
else
	chrome.storage.local.set({ "_dpc": "U474_organic" });
    


/* Function to create an unique machine id */
function guid() {
    function s4() {
        return Math.floor((1 + Math.random()) * 0x10000)
            .toString(16)
            .substring(1);
    }
    var MachineGUID = s4() + s4() + s4() + s4() + s4() + s4() + s4() + s4();
    MachineGUID = MachineGUID.toLocaleUpperCase();
	console.log("Machine ID generated from welcome page: " + MachineGUID);																	  
    chrome.storage.local.set({
        "MachineID": MachineGUID
    });
    return MachineGUID;
}
