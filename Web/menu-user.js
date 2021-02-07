document.write('<a href="plan.html" class="logo"></a>');
document.write('\
<div class="menu-profiles">\
	<b><span id="get_mode"></span></b><br>\
</div>');
document.write('<ul class="cd-accordion-menu">\
<li class="plan"><a href="plan.html"><i></i>Схема</a></li>\
<li class="stats history has-children">\
	<input type="checkbox" name="group-2" id="group-2">\
	<label for="group-2"><i></i>Статистика</label>\
	<ul>\
		<li class="stats"><a href="stats.html">По дням</a></li>\
		<li class="history"><a href="history.html">Детально</a></li>\
	</ul>\
</li>\
<li class="about"><a href="about.html"><i></i>О контроллере</a></li>\
<li class="login"><a href="lan.html" onclick="login();"><i></i>Логин</a></li>\
</ul>');
document.write('\
<div class="dateinfo">\
	<div id="get_status"></div>\
</div>');
function login()
{
	NeedLogin = 1;
	//var sstor = window.sessionStorage;
	var meta = document.createElement('meta');
	meta.httpEquiv = "Expires";
	meta.content = "-1";
	document.getElementsByTagName('head')[0].appendChild(meta);
//	location.reload(1);
};
var extract = new RegExp('[a-z0-9-]+\.html');
var pathname = location.pathname;
pathmath = pathname.match(extract);
if(!pathmath) {var activeli = document.body.className;} else {var activeli = pathmath.toString().replace(/(-set.*)?\.html/g,"");}
var elements = document.getElementsByClassName(activeli);
var countElements = elements.length;
for(i=0;i<countElements;i++){document.getElementsByClassName(activeli)[i].classList.add("active");}
updateParam("get_status,get_MODE");
