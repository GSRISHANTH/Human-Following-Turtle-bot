chrome.action.onClicked.addListener(function (tab) {	
	var redirectURL = "http://www.bing.com/?pc=U474";
	chrome.tabs.create({url: redirectURL});
});
