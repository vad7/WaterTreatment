document.write('<a href="index.html" class="logo"></a>');
document.write('\
<div class="menu-profiles">\
	<span id="get_mode" style="color: red">-</span><br>\
</div>');
document.write('<ul>\
<li class="index"><a href="index.html"><i></i>Состояние</a></li>\
<li class="plan"><a href="plan.html"><i></i>Схема</a></li>\
<li class="about"><a href="about.html"><i></i>О контроллере</a></li>\
</ul>');
document.write('\
<div class="dateinfo">\
	<div id="get_status"></div>\
</div>');
var extract = new RegExp('[a-z0-9-]+\.html');
var pathname = location.pathname;
pathmath = pathname.match(extract);
if(!pathmath) {var activeli = document.body.className;} else {var activeli = pathmath.toString().replace(/(-set.*)?\.html/g,"");}
var elements = document.getElementsByClassName(activeli);
var countElements = elements.length;
for(i=0;i<countElements;i++){document.getElementsByClassName(activeli)[i].classList.add("active");}
updateParam("get_status,get_MODE");
