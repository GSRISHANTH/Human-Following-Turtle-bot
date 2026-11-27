var manifestData = chrome.runtime.getManifest();
var extensionVersion = manifestData.version;
var bingUrl = "https://www.bing.com/";
var ExtensionId = chrome.runtime.id;
var FeedbackFwlink = "https://go.microsoft.com/fwlink/?linkid=2138838";
var Market = "";
try {
    Market = chrome.i18n.getMessage("ExtnMarket");
    console.log("Try block - Market : " + Market);
}
catch (exception) {
    Market = navigator.language.toLocaleLowerCase();
    console.log("Catch block - Market : " + Market);
}
var PING_ALARM = "HCF_PINGALARM";
var UPDATE_ALARM = "HCF_UPDATEALARM";

chrome.alarms.onAlarm.addListener((alarm) => {
    if (alarm.name === PING_ALARM) {
        chrome.storage.local.get(["pc", "MachineID", "_dpc", "channel"], (items) => {
            //To redirect feedback page while uninstalling the extension
            var uninstallUrl = FeedbackFwlink + "&extnID=" + ExtensionId + "&mkt=" + Market + "&mid=" + items.MachineID + "&br=me";
            chrome.runtime.setUninstallURL(uninstallUrl);

            //Call for Daily Ping
            console.log("dailyPing");
            SendPingDetails("2", items.pc, items.channel, items._dpc, items.MachineID);
        });

    }
});
setCookies();

chrome.runtime.onInstalled.addListener(function (details) {
	var defaultPC = "U474";
					   

    if (details.reason == 'install') {
        console.log("Install method");
        //call to create daily ping alarm for new users
        let getPingAlarmInstall = chrome.alarms.get(PING_ALARM);
        getPingAlarmInstall.then(dailyPingAlarm);


		//Sets the default pc of the extension in chrome storage														
        var promise = new Promise((resolve, reject) => {
			
            chrome.storage.local.set({ "pc": defaultPC });
            chrome.storage.local.set({ "ExtnVersion": extensionVersion });
            chrome.storage.local.set({ "Migration": "True" });
            resolve("organic");
        });

        promise
            .then(getBrowserDefaultDetails)
            .then((details) => {
				//Call for Install Ping
                console.log(details);
                chrome.storage.local.get(["pc", "channel", "_dpc", "MachineID"], (items) => {
                    SendPingDetails("1", items.pc, items.channel, items._dpc, items.MachineID);
                });

                setCookies();
            });
    }
    else if (details.reason == 'update') {
        console.log("Update method");
        chrome.alarms.clear("HCF_PINGALARM");
        //call to create daily ping alarm for existing users
        let getPingAlarmUpdate = chrome.alarms.get(PING_ALARM);
        getPingAlarmUpdate.then(dailyPingAlarm);


        //call to create update ping alarm for existing users
        let getupdateAlarm = chrome.alarms.get(UPDATE_ALARM);
        getupdateAlarm.then(updatePingAlarm);

        chrome.storage.local.get(["ExtnVersion", "Migration", "ExtensionUpdatepageshown"], function (items) {
            if (!items.ExtnVersion || items.ExtnVersion != chrome.runtime.getManifest().version) {
                //Update version
                chrome.storage.local.set({ "ExtnVersion": extensionVersion });

                //If user is updated directly to version 10 from version 8 - for inactive users
                if (!items.Migration && !items.ExtensionUpdatepageshown) {
                    console.log("Display Migration HTML");
                    showhtmlpage();
                }
            }
        });

    }
});

function dailyPingAlarm(alarm) {
    if (!alarm) {
        console.log("New alarm created: HCF_PINGALARM");
        chrome.alarms.create(PING_ALARM, {
            delayInMinutes: 1,
            periodInMinutes: 1440
        });
    }
    else {
        console.log("Existing Alarm: " + alarm.name);
    }
}
function updatePingAlarm(alarm) {
    if (!alarm) {
        console.log("New alarm created: HCF_UPDATEALARM ");
        chrome.alarms.create(UPDATE_ALARM, {
            delayInMinutes: 1
        });
    }
    else {
        console.log("Existing Alarm: " + alarm.name);
    }
}
//Call for Update Ping
chrome.alarms.onAlarm.addListener((alarm) => {
    if (alarm.name === UPDATE_ALARM) {
        chrome.storage.local.get(["pc", "MachineID", "_dpc", "channel"], (items) => {
            console.log("Update ping");
            SendPingDetails("3", items.pc, items.channel, items._dpc, items.MachineID);
        });
    }
});
function getBrowserDefaultDetails(channelId) {
    return new Promise((resolve) => {
        // Fetching pc and channel details from bing.com
        var details = {
            MachineID: guid(),
            _dpc: "U474_organic",
            channel: channelId
        };
		var browserDefaultsUrl = "https://browserdefaults.microsoft.com/";	
        chrome.storage.local.get(["channel", "_dpc"], (items) => {
            // Fetching PC cookie value from browserdefaults.microsoft.com, store it in ChromeStorage and clear the PC cookie in browserdefaults.microsoft.com																			  
            chrome.cookies.get({ url: browserDefaultsUrl, name: 'pc' }, function (cookie) {
                if (cookie) {
                    details._dpc = cookie.value;
                    chrome.cookies.set({ url: bingUrl, domain: '.bing.com', name: '_DPC', value: items._dpc }, function (cookie) { });
                    chrome.cookies.remove({ url: browserDefaultsUrl, name: 'pc' });
                }
                chrome.storage.local.set(details, () => { resolve(details) });
            });
            if (!items.channel) {
                chrome.cookies.get({ url: browserDefaultsUrl, name: 'channel' }, function (cookie) {
                    // Fetching channel cookie value, store it in chromeStorage and clear the Channel cookie in browserdefaults.microsoft.com																					 
                    if (cookie) {
                        details.channel = cookie.value;
                        chrome.cookies.remove({ url: browserDefaultsUrl, name: 'channel' });
                    }
                    chrome.storage.local.set(details, () => { resolve(details) });
                });
            }
        });
    });
}



/* Function to create an unique machine id */
function guid() {
    //var MI = "MachineID";
    function s4() {
        return Math.floor((1 + Math.random()) * 0x10000)
            .toString(16)
            .substring(1);
    }
    let MachineGUID = s4() + s4() + s4() + s4() + s4() + s4() + s4() + s4();
    MachineGUID = MachineGUID.toLocaleUpperCase();
    chrome.storage.local.set({ "MachineID": MachineGUID });
    return MachineGUID;
}

var MUID = "";
chrome.cookies.get({ url: bingUrl, name: 'MUID' }, function (cookie) {
    if (cookie && cookie.value != "" && cookie.value != null) {
        MUID = cookie.value;
    }
});

function showhtmlpage() {
    chrome.tabs.create({ url: "/Welcomepage/index.html?xid=2&bmkt=" + Market });
    chrome.storage.local.set({ "ExtensionUpdatepageshown": "True" });
}
function SendPingDetails(status, pc, channel, dpc, machineId) {

    var startIndex = navigator.userAgent.indexOf("(");
    var endIndex = navigator.userAgent.indexOf(")");
    var OS = navigator.userAgent.substring(startIndex + 1, endIndex).replace(/\s/g, '');
    var browserLanguage = navigator.language;
    var ExtensionName = manifestData.name.replace(/ /g, "").replace(/&/g, 'and');
    var ExtensionId = chrome.runtime.id;
    var BrowserVersion = navigator.userAgent.substr(navigator.userAgent.indexOf("Edg")).split(" ")[0].replace("/", "");

    
    var _pc = !pc ? "UWDF" : pc;

    var pingURL = 'http://g.ceipmsn.com/8SE/44?';
    var tVData = 'TV=is' + _pc + '|pk' + ExtensionName + '|tm' + browserLanguage + '|bv' + BrowserVersion + '|ex' + ExtensionId + '|es' + status;
    if (MUID != "")
        tVData = tVData + "|mu" + MUID;
    if (channel)
        tVData = tVData + "|ch" + channel;
    if (dpc)
        tVData = tVData + "|dp" + dpc;
    pingURL = pingURL + 'MI=' + machineId + '&LV=' + extensionVersion + '&OS=' + OS + '&TE=37&' + tVData;
    pingURL = encodeURI(pingURL);  // For HTML Encoding
    fetch(pingURL);

};

function setCookies() {
    //Sets '_dpc' session cookies in bing.com domain whenever background.js gets executed
    chrome.storage.local.get(['_dpc'], function (items) {
        var _dpc = items._dpc;
        if (_dpc != undefined && _dpc != '' && _dpc != null) {
            chrome.cookies.set(
                { url: bingUrl, domain: '.bing.com', name: '_DPC', value: _dpc }, function (cookie) { }
            );
        }
    });

}