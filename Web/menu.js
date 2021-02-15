document.write('<a href="plan.html" class="logo"></a>');
document.write('<br>\
<div class="menu-profiles">\
	<b><span id="get_mode"></span></b><br>\
</div>');
document.write('\
<ul class="cd-accordion-menu">\
<li class="plan"><a href="plan.html"><i></i>Схема</a></li>\
<li class="stats history has-children">\
	<input type="checkbox" name="group-2" id="group-2">\
	<label for="group-2"><i></i>Статистика</label>\
	<ul>\
		<li class="stats"><a href="stats.html">По дням</a></li>\
		<li class="history"><a href="history.html">Детально</a></li>\
	</ul>\
</li>\
<li name="USR"  class="setsensors sensorst sensorsp relay has-children">\
	<input type="checkbox" name="group-3" id="group-3">\
	<label for="group-3"><i></i>Конфигурация</label>\
	<ul>\
		<li class="sensorsp"><a href="sensorsp.html">Датчики</a></li>\
		<li class="relay"><a href="relay.html">Датчики, Реле</a></li>\
		<li class="sensorst"><a href="sensorst.html">Датчики температуры</a></li>\
		<li class="setsensors"><a href="setsensors.html">Привязка датчиков</a></li>\
	</ul>\
</li>\
<li name="USR" class="lan config system files time notice mqtt const has-children">\
	<input type="checkbox" name="group-4" id="group-4">\
	<label for="group-4"><i></i>Сервис</label>\
	<ul>\
		<li class="config"><a href="config.html">Настройки</a></li>\
		<li class="lan"><a href="lan.html">Сеть</a></li>\
		<li class="time"><a href="time.html">Дата/Время</a></li>\
		<li class="notice"><a href="notice.html">Уведомления</a></li>\
		<li class="mqtt"><a href="mqtt.html">MQTT</a></li>\
		<li class="system"><a href="system.html">Система</a></li>\
		<li class="const"><a href="const.html"><i></i>Константы</a></li>\
		<li class="files"><a href="files.html">Файлы</a></li>\
	</ul>\
</li>\
<li name="USR" class="charts test modbus log freertos has-children">\
	<input type="checkbox" name="group-5" id="group-5">\
	<label for="group-5"><i></i>Отладка</label>\
	<ul>\
		<li class="charts"><a href="charts.html">Графики</a></li>\
		<li class="test"><a href="test.html">Тестирование</a></li>\
		<li class="modbus"><a href="modbus.html">Modbus</a></li>\
		<li class="log"><a href="log.html">Журнал</a></li>\
		<li class="freertos"><a href="freertos.html">ОС RTOS</a></li>\
	</ul>\
</li>\
<li class="about"><a href="about.html"><i></i>О контроллере</a></li>\
<li name="USR" id="mlogin" hidden><a href="planf.html" onclick="NeedLogin=1"><i></i>Логин</a></li>\
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
loadParam("USR");
updateParam("get_status,get_MODE");
window.onscroll = function() { autoheight(); }
