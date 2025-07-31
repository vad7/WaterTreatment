// Copyright by Vadim Kulakov vad7@yahoo.com, vad711
var VER_WEB = "1.73";
var urlcontrol = ''; //  автоопределение (если адрес сервера совпадает с адресом контроллера)
// адрес и порт контроллера, если адрес сервера отличен от адреса контроллера (не рекомендуется)
//var urlcontrol = 'http://192.168.0.199';
//var urlcontrol = 'http://192.168.0.8';
var urltimeout = 1800; // таймаут ожидание ответа от контроллера. Чем хуже интернет, тем выше значения. Но не более времени обновления параметров
var urlupdate = 4000; // время обновления параметров в миллисекундах
//if(navigator.userAgent.indexOf("rk3568_r109")!=-1) { }

function setParam(paramid, resultid) {
	// Замена set_Par(Var1) на set_par-var1 для получения значения 
	var elid = paramid.replace("(", "-").replace(")", "").toLowerCase();
	var rec = new RegExp('et_listChart.?');
	var res = new RegExp('et_testMode|et_mode');
	var elval, clear = true, equate = true;
	var element;
	if((clear = equate = elid.indexOf("=")==-1)) { // Не (x=n)
		if((element = document.getElementById(elid))); else element = document.getElementById(resultid);
		if(element.getAttribute('type') == 'checkbox') {
			if(element.checked) elval = 1; else elval = 0;
		} else elval = element.value;
		//if(typeof elval == 'string') elval = elval.replace(/[,=&]+/g, "");
	}
	if(res.test(paramid)) {
		var elsend = paramid.replace("get_", "set_").replace(")", "") + "(" + elval + ")";
	} else if(rec.test(paramid)) {
		var elsend = paramid.replace("get_listChart", "get_Chart(" + elval + ")");
		clear = false;
	} else {
		var elsend = paramid.replace("get_", "set_");
		if(equate) {
			if(elsend.substr(-1) == ")") elsend = elsend.replace(")", "") + "=" + elval + ")"; else elsend += "=" + elval;
		}
	}
	if(!resultid) resultid = elid.replace("set_", "get_");
	if(clear) {
		element = document.getElementById(resultid);
		if(element) {
			element.value = "";
			element.placeholder = "";
		}
	}
	loadParam(encodeURIComponent(elsend.replace(/&/g,'$')), true, resultid);
}

var NeedLogin = sessionStorage.getItem("NeedLogin");
var LPString = sessionStorage.getItem("LPString");

function loadParam(paramid, noretry, resultdiv) {
	var check_ready = 1;
	var queue = 0;
	var req_stek = new Array();
	if(queue == 0) {
		req_stek.push(paramid);
	} else if(queue == 1) {
		req_stek.unshift(paramid);
		queue = 0;
	}
	if(check_ready == 1) {
		unique(req_stek);
		var oneparamid = req_stek.shift();
		check_ready = 0;
		var request = new XMLHttpRequest();
		var reqstr = urlcontrol + "/&" + oneparamid.replace(/,/g, '&') + "&&";
		if(NeedLogin) {
			if(NeedLogin != 2) {
				LPString = "!Z" + window.btoa(prompt("Login:") + ":" + prompt("Password:"));
				NeedLogin = 2;
				sessionStorage.setItem("NeedLogin", NeedLogin);
				sessionStorage.setItem("LPString", LPString);
			}
			reqstr += LPString;
		} 
		request.open("GET", reqstr, true);
		request.timeout = urltimeout;
		request.send(null);
		request.onreadystatechange = function() {
			if(this.readyState != 4) return;
			if(request.status == 200) {
				if(request.responseText != null) {
					var arr = request.responseText.replace(/^\x7f+/, '').replace(/\x7f\x7f*$/, '').split('\x7f');
					if(arr != null && arr != 0) {
						check_ready = 1; // ответ получен, можно слать следующий запрос.
						if(req_stek.length != 0) // если массив запросов не пуст - заправшиваем следующие значения.
						{
							queue = 1;
							setTimeout(function() {	loadParam(req_stek.shift()); }, 10); // запрашиваем следующую порцию.
						}
						for(var i = 0; i < arr.length; i++) {
							if(arr[i] != null && arr[i] != 0) {
								values = arr[i].split('=');
								var valueid = values[0].replace("(", "-").replace(")", "").replace("set_", "get_").toLowerCase();
								var type, element;
								if(/get_status|get_sys|^CONST|get_socketInfo/.test(values[0])) type = "const"; 
								else if(/_list|\(RULE|et_testMode|\(TARGET|NSL/.test(values[0])) type = "select"; // значения
								else if(/get_tbl|listRelay|get_numberIP|TASK_/.test(values[0])) type = "table"; 
								else if(values[0].indexOf("get_is")==0) type = "is"; // наличие датчика в конфигурации
								else if(values[0].indexOf("scan_")!=-1) type = "scan"; // ответ на сканирование
								else if(values[0].indexOf("hide_")==0) { // clear
									if(values[1] == 1) {
										var elements = document.getElementsByName(valueid);
										for(var j = 0; j < elements.length; j++) elements[j].innerHTML = "";
									}
									continue;
								} else if(values[0].indexOf("get_Chart")==0) type = "chart"; // график
								else if(values[0].indexOf("et_modbus_v")==1) type = "tbv"; // таблица значений
								else if(/LvL[()]/.test(values[0]) && !!(element = document.getElementById(valueid)).getAttribute("name")) type = "bar";
								else if(values[0].indexOf("RELOAD")==0) { 
									location.reload();
								} else if(values[0].indexOf("(DSD")!=-1) {
									loadParam("get_tblPDS");
								} else {
									if(values[0].indexOf("CMD_")==0) valueid = values[0].substring(0, values[0].indexOf("(")).toLowerCase();
									if((element = document.getElementById(valueid + "-ONOFF"))) { // Надпись
										if(values[1].match(/^E-?\d|^ERR/)) element.innerHTML = values[1];
										else element.innerHTML = values[1] == 1 ? "Вкл" : "Выкл";
									}
									element = document.getElementById(valueid);
									if(element && element.getAttribute('type') == 'checkbox') {
										if(values[1].match(/^E-?\d|^ERR/)) {
											if(values[0].substr(0, 3) == "set") element.checked = !element.checked;
											var element2 = document.getElementById(valueid + '-LBL');
											if(element2) element2.textContent = values[1];
											else element.insertAdjacentHTML('afterend', '<label id="' + valueid + '-L" for="' + valueid + '">' + values[1] + '</label>');
										} else {
											var onoff = values[1] == 1;
											element.checked = onoff;
											if((element = document.getElementById(valueid + '-LBL'))) element.textContent = "";
											var elements = document.getElementsByName(valueid + "-hide");
											for(var j = 0; j < elements.length; j++) elements[j].style = "display:" + (onoff ? "inline" : "none");
											var elements = document.getElementsByName(valueid + "-unhide");
											for(var j = 0; j < elements.length; j++) elements[j].style = "display:" + (!onoff ? "inline" : "none");
										}
										continue;
									} 
									type = /\([a-z0-9_]+\)/i.test(values[0]) ? "values" : "str";
								}
								if(type == 'chart') {
									if(values[0]) {
										if(!window.time_chart) { console.log("Chart was not intialized!"); continue; }
										createChart(values, resultdiv);
									}
								} else if(type == 'scan') {
									if(valueid == "get_message-scan_sms") {
										if(values[1] == "wait response") {
											setTimeout(loadParam('get_Message(scan_SMS)'), 2000);
										} else alert(values[1]);
									} else if(valueid == "get_message-scan_mail") {
										if(values[1] == "wait response") {
											setTimeout(loadParam('get_Message(scan_MAIL)'), 2000);
										} else alert(values[1]);
									} else {
										var cont2 = [];
										if(values[0] != null && values[0] != 0 && values[1] != null && values[1] != 0) {
											var content = "<tr><td>" + values[1].replace(/\:/g, "</td><td>").replace(/(\;)/g, "</td></tr><tr><td>") + "</td></tr>";
											document.getElementById(values[0].toLowerCase()).innerHTML = content;
											content = values[1].replace(/:[^;]+/g, "").replace(/;$/g, "");
											cont2 = content.split(';');
										}
										var elems = document.getElementById("scan_table").getElementsByTagName('select');
										for(var j = 0; j < elems.length; j++) {
											elems[j].options.length = 0
											elems[j].add(new Option("---", "", true, true), null);
											elems[j].add(new Option("reset", 0, false, false), null);
											for(var k = 0; k < cont2.length; k++) {
												elems[j].add(new Option(k + 1, k + 1, false, false), null);
											}
										}
									}
								} else if(type == 'select') {
									if(values[0] != null && values[0] != 0 && values[1] != null) {
										var idsel = values[0].replace("set_", "get_").toLowerCase().replace(/\([0-9]\)/g, "").replace("(", "-").replace(")", "").replace(/_skip[1-9]$/,"");
										if(idsel.substr(-1, 1) == '_') {
											idsel = idsel.substring(0, idsel.length - 1);
											element = document.getElementById(idsel);
											if(element) {
												var n = (Number(values[1]) + 1).toString() + '.';
												for(var j = 0; j < element.options.length; j++) if(n == element.options[j].innerText.substr(0, n.length)) { element.options[j].selected = true; break; }
											}
											continue;
										}
										if(idsel == "get_testmode") {
											var element2 = document.getElementById("get_testmode2");
											if(element2) {
												var cont1 = values[1].split(';');
												for(var n = 0, len = cont1.length; n < len; n++) {
													var cont2 = cont1[n].split(':');
													if(cont2[1] == 1) {
														if(cont2[0] != "NORMAL") {
															document.getElementById("get_testmode2").className = "red";
														} else {
															document.getElementById("get_testmode2").className = "";
														}
														document.getElementById("get_testmode2").innerHTML = cont2[0];
													}
												}
											}
										} else if(idsel == "get_listadc") {
											content = "";
											content2 = "";
											upsens = "";
											loadsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												var P = count[j];
												loadsens += "get_zADC(" +P+ "),get_tADC(" +P+ "),get_maxADC(" +P+ "),get_minADC(" +P+ "),get_minrADC(" +P+ "),get_pinADC(" +P+ "),get_GADC(" +P+ "),get_nADC(" +P+ "),get_testADC(" +P+ "),";
												upsens += "get_ADC(" +P+ "),get_adcADC(" +P+ "),";
												P = P.toLowerCase();
												content += '<tr id="get_isadc-' +P+ '">';
												content += '<td>' +count[j]+ '</td>';
												content += '<td id="get_nadc-' +P+ '"></td>';
												content += '<td id="get_adc-' +P+ '" nowrap>-</td>';
												content += '<td nowrap><input id="get_minadc-' +P+ '" type="number" step="0.01"><input type="submit" value=">" onclick="setParam(\'get_minADC(' +count[j]+ ')\');"></td>';
												content += '<td nowrap><input id="get_maxadc-' +P+ '" type="number" step="0.01"><input type="submit" value=">" onclick="setParam(\'get_maxADC(' +count[j]+ ')\');"></td>';
												content += '<td nowrap><input id="get_zadc-' +P+ '" type="number" step="1"><input type="submit" value=">" onclick="setParam(\'get_zADC(' +count[j]+ ')\');"></td>';
												content += '<td nowrap><input id="get_tadc-' +P+ '" type="number" step="0.001"><input type="submit" value=">" onclick="setParam(\'get_tADC(' +count[j]+ ')\');"></td>';
												content += '<td nowrap><input id="get_gadc-' +P+ '" type="number" step="1"><input type="submit" value=">" onclick="setParam(\'get_GADC(' +count[j]+ ')\');"></td>';
												content += '<td nowrap><input id="get_testadc-' +P+ '" type="number" step="0.01"><input type="submit" value=">" onclick="setParam(\'get_testADC(' +count[j]+ ')\');"></td>';
												content += '<td id="get_adcadc-' +P+ '">-</td>';
												content += '<td id="get_pinadc-' +P+ '">-</td>';
												content += '</tr>';
											}
											document.getElementById(idsel + "2").innerHTML = content;
											updateParam(upsens);
											loadParam(loadsens);
											values[1] = "--;" + values[1];
										}
										element = document.getElementById(idsel);
										if(element) {
										  if(values[0].substr(-6, 5) == "_skip") {
											var j2 = Number(values[0].substr(-1)) - 1;
											for(var j = element.options.length - 1; j > j2; j--) element.options[j].remove();
										  } else element.innerHTML = "";
										  var cont1 = values[1].split(';');
										  if(element.tagName != "SPAN") {
											for(var k = 0; k < cont1.length - 1; k++) {
												var cont2 = cont1[k].split(':');
												if(cont2[1] == 1) {
													selected = true;
												} else selected = false;
												if(idsel == "get_listchart") {
													var elems = document.getElementsByName("chrt_sel");
													for(var j = 0; j < elems.length; j++) {
														elems[j].add(new Option(cont2[0],cont2[0], false, selected), null); // "_"+
													}
												} else {
													var opt = new Option(cont2[0], k, false, selected);
													if(cont2[2] == 0) opt.disabled = true;
													element.add(opt, null);
												}
											}
										  } else {
											for(var k = 0; k < cont1.length - 1; k++) {
												var cont2 = cont1[k].split(':');
												if(cont2[1] == 1) { element.innerHTML = cont2[0]; break; } 
											}
										  }
										}
									}
								} else if(type == 'const') {
									var element = document.getElementById(valueid);
									if(element && values[0] != null && values[0] != 0 && values[1] != null && values[1] != 0) {
										if(values[0] == 'get_status') {
											element.innerHTML = "<div>" + values[1].replace(/>/g, "'><nobr>").replace(/\|/g, "</nobr></div><div id='") + "</div>";
										} else {
											if(values[0] == "CONST") values[1] = "VER_WEB|Версия веб-страниц|" + VER_WEB + ';' + values[1];
											element.innerHTML = "<tr><td>" + values[1].replace(/\|/g, "</td><td>").replace(/(\;)/g, "</td></tr><tr><td>") + "</td></tr>";
										}
									}
								} else if(type == 'table') {
									if(values[0] == 'get_tblErr') {
										element = document.getElementById(valueid + "_t");
										if(element) element.hidden = values[1] == "";
									}
									if(values[1] != null && values[1] != 0) {
										if(values[0] == 'get_tblInput') {
											var content = "", content2 = "", upsens = "", loadsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												input = count[j].toLowerCase();
												loadsens = loadsens + "get_alarmInput(" + count[j] + "),get_eInput(" + count[j] + "),get_typeInput(" + count[j] + "),get_pinInput(" + count[j] + "),get_Input(" + count[j] + "),get_nInput(" + count[j] + "),get_testInput(" + count[j] + "),";
												upsens = upsens + "get_Input(" + count[j] + "),get_eInput(" + count[j] + "),";
												content = content + '<tr><td>' + count[j] + '</td><td id="get_ninput-' + input + '"></td><td id="get_input-' + input + '">-</td> <td nowrap><input id="get_alarminput-' + input + '" type="number" min="0" max="1"><input type="submit" value=">" onclick="setParam(\'get_alarmInput(' + count[j] + ')\');"></td><td nowrap><input id="get_testinput-' + input + '" type="number" min="0" max="1" step="1" value=""><input type="submit" value=">"  onclick="setParam(\'get_testInput(' + count[j] + ')\');"></td><td id="get_pininput-' + input + '">-</td><td id="get_typeinput-' + input + '">-</td><td id="get_einput-' + input + '">-</td>';
												content = content + '</tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
											loadParam(loadsens);
										} else if(values[0] == 'get_tblTempF') {
											var content = "", upsens = "", loadsens = "";
											var tnum = 1;
											element = document.getElementById(valueid);
											if(!element) {
												element = document.getElementById(valueid + '2');
												if(element) tnum = 2; // set
											}
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												var T = count[j];
												loadsens += "get_eTemp(" +T+ "),";
												upsens += "get_eTemp(" +T+ "),";
												if(tnum == 1) {
													loadsens += "get_maxTemp(" +T+ "),get_erTemp(" +T+ "),get_esTemp(" +T+ "),get_minTemp(" +T+ "),get_fTemp4(" +T+ "),get_fTemp5(" +T+ "),get_fTemp6(" +T+ "),get_nTemp(" +T+ "),get_testTemp(" +T+ "),get_bTemp(" +T+ "),";
													upsens += "get_fullTemp(" +T+ "),get_esTemp(" +T+ "),";
												} else if(tnum == 2) {
													loadsens += "get_aTemp(" +T+ "),get_fTemp1(" +T+ "),get_fTemp2(" +T+ "),get_fTemp3(" +T+ "),get_nTemp2(" +T+ "),get_bTemp(" +T+ "),";
													upsens += "get_rawTemp(" +T+ "),";
												}
												T = T.toLowerCase();
												content += '<tr>';
												content += '<td>' +count[j]+ '</td>';
												if(tnum == 1) {
													content += '<td id="get_ntemp-' +T+ '"></td>';
													content += '<td id="get_fulltemp-' +T+ '">-</td>';
													content += '<td id="get_mintemp-' +T+ '">-</td>';
													content += '<td id="get_maxtemp-' +T+ '">-</td>';
													content += '<td nowrap><input id="get_ertemp-' +T+ '" type="number" step="0.01"><input type="submit" value=">" onclick="setParam(\'get_erTemp(' + count[j] + ')\');"></td>';
													content += '<td nowrap><input id="get_testtemp-' +T+ '" type="number" step="0.1"><input type="submit" value=">" onclick="setParam(\'get_testTemp(' + count[j] + ')\');"></td>';
													//content += '<td nowrap><input type="checkbox" id="get_ftemp4-' +T+ '" onchange="setParam(\'get_fTemp4(' +count[j]+')\');"><input type="checkbox" id="get_ftemp5-' +T+ '" onchange="setParam(\'get_fTemp5(' +count[j]+')\');"><input type="checkbox" id="get_ftemp6-' +T+ '" onchange="setParam(\'get_fTemp6(' +count[j]+')\');"></td>';
													content += '<td id="get_btemp-' +T+ '">-</td>';
													content += '<td id="get_estemp-' +T+ '">-</td>';
												} else if(tnum == 2) {
													content += '<td id="get_ntemp2-' +T+ '"></td>';
													content += '<td id="get_rawtemp-' +T+ '">-</td>';
													content += '<td nowrap><span id="get_btemp-' +T+ '">-</span>:<span id="get_atemp-' +T+ '">-</span></td>';
													content += '<td><select id="set_atemp-' +T+ '" onchange="setParam(\'set_aTemp(' +count[j]+ ')\');"></select></td>';
													content += '<td nowrap><input type="checkbox" id="get_ftemp1-' +T+ '" onchange="setParam(\'get_fTemp1(' +count[j]+')\');"><input type="checkbox" id="get_ftemp2-' +T+ '" onchange="setParam(\'get_fTemp2(' +count[j]+')\');"><input type="checkbox" id="get_ftemp3-' +T+ '" onchange="setParam(\'get_fTemp3(' +count[j]+ ')\');"></td>';
												}
												content += '<td id="get_etemp-' +T+ '">-</td>';
												content += '</tr>';
												if(loadsens.length >= 1200) {
													loadParam(loadsens);
													loadsens = "";
												}
											}
											element.innerHTML = content;
											if(loadsens.length) loadParam(loadsens);
											updateParam(upsens);
										} else if(values[0].substr(0, 11) == 'get_tblTemp') {
											var content = "", loadsens = "", upsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												var T = count[j];
												loadsens += "get_nTemp(" +T+ "),";
												upsens += "get_fullTemp(" +T+ "),";
												T = T.toLowerCase();
												content += '<tr><td nowrap><span id="get_ntemp-' +T+ '"></span>:</td><td id="get_fulltemp-' +T+ '"></td></tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											loadParam(loadsens);
											updateParam(upsens);
										} else if(values[0] == 'get_tblFlow') {
											var content = "", upsens = "", loadsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												input = count[j].toLowerCase();
												loadsens = loadsens + "get_nFlow(" + count[j] + "),get_kfFlow(" + count[j] + "),get_kNLFlow(" + count[j] + "),get_cFlow(" + count[j] + "),get_frFlow(" + count[j] + "),get_testFlow(" + count[j] + "),get_pinFlow(" + count[j] + "),get_eFlow(" + count[j] + "),";
												//,get_I2CFlow(" + count[j] + "),";
												upsens = upsens + "get_Flow(" + count[j] + "),get_rFlow(" + count[j] + "),get_frFlow(" + count[j] + "),get_eFlow(" + count[j] + "),";
												content = content + '<tr>';
												content = content + '<td>' + count[j] + '</td>';
												content = content + '<td id="get_nflow-' + input + '">-</td>';
												content = content + '<td nowrap><span id="get_flow-' + input + '">-</span> <input id="ClcFlow' + input + '" type="submit" value="*" onclick="CalcAvgValue(\''+ input + '\')"></td>';
												content = content + '<td><span id="get_rflow-' + input + '">-</span></td>';
												content = content + '<td id="get_frflow-' + input + '">-</td>';
												content = content + '<td nowrap><input id="get_kfflow-' + input + '" type="number" step="0.01" style="max-width:70px;" value=""><input type="submit" value=">" onclick="setParam(\'get_kfFlow(' + count[j] + ')\');"></td>';
												content = content + '<td nowrap><input id="get_knlflow-' + input + '" type="number" step="0.01" style="max-width:70px;" value=""><input type="submit" value=">" onclick="setParam(\'get_kNLFlow(' + count[j] + ')\');"></td>';
												content = content + '<td nowrap><input id="get_cflow-' + input + '" type="number"><input type="submit" value=">" onclick="setParam(\'get_cFlow(' + count[j] + ')\');"></td>';
												content = content + '<td nowrap><input id="get_testflow-' + input + '" type="number" step="0.001" value=""><input type="submit" value=">" onclick="setParam(\'get_testFlow(' + count[j] + ')\');"></td>';
												content = content + '<td id="get_eflow-' + input + '">-</td>';
												content = content + '<td id="get_pinflow-' + input + '">-</td>';
												//content = content + '<td nowrap><input id="get_i2cflow-' + input + '" type="number"><input type="submit" value=">" onclick="setParam(\'get_I2CFlow(' + count[j] + ')\');"></td>';
												content = content + '</tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
											loadParam(loadsens);
										} else if(values[0] == 'get_tblFlowС') {
											var content = "", upsens = "get_ADC(PWATER),", loadsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												input = count[j].toLowerCase();
												upsens = upsens + "get_CEFlow(" + count[j] + "),get_CLFlow(" + count[j] + "),get_CKFlow0(" + count[j] + "),get_CKFlow1(" + count[j] + "),";
												content = content + '<tr>';
												content = content + '<td>' + count[j] + '</td>';
												content = content + '<td><span id="get_ceflow-' + input + '">-</span></td>';
												content = content + '<td><span id="get_clflow-' + input + '">-</span></td>';
												content = content + '<td><span id="get_ckflow0-' + input + '">-</span></td>';
												content = content + '<td><input type="submit" value="Сброс" onclick="loadParam(\'get_zFlow(' + count[j] + ')\')"></td>';
												content = content + '<td></td>';
												content = content + '</tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
										} else if(values[0] == 'get_tblRelay') {
											var content = "", upsens = "", loadsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												if((relay = count[j].toLowerCase()) == "") continue;
												loadsens = loadsens + "get_pinRelay(" + count[j] + "),get_isRelay(" + count[j] + "),get_nRelay(" + count[j] + "),";
												upsens = upsens + "get_Relay(" + count[j] + "),";
												content = content + '<tr id="get_isrelay-' + relay + '"><td>' + count[j] + '</td><td id="get_nrelay-' + relay + '"></td><td><span id="get_relay-' + relay + '-ONOFF"></span><input type="checkbox" name="relay" id="get_relay-' + relay + '" onchange="setParam(\'get_Relay(' + count[j] + ')\');"></td><td id="get_pinrelay-' + relay + '"></td>';
												content = content + '</tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
											loadParam(loadsens);
										} else if(values[0] == 'get_tblRelayMdb') {
											var content = "", upsens = "";
											var count = values[1].split(';');
											for(var j = 0; j < count.length - 1; j++) {
												var flds = count[j].split('|');
												if(flds.length == 0) continue;
												if((relay = flds[0]) == "") continue;
												upsens = upsens + "get_modbusR(" + relay + "),get_modbusE(" + relay + "),";
												content = content + '<tr><td>' + flds[1] + '</td><td id="get_modbusr-' + relay + '-ONOFF"></td><td>' + flds[2] + '</td><td>' + flds[3] + '</td><td id="get_modbuse-' + relay + '"></td>';
												content = content + '</tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
										} else if(values[0] == 'get_tblPwrC') {
											var content = "";
											var count = values[1].split(';');
											if(count.length <= 1) {
												element = document.getElementById(valueid + "_head");
												if(element) element.innerHTML = content;
											} else {
												for(var j = 0; j < count.length - 1; j++) {
													content = content + '<tr><td>' + count[j] + '</td><td nowrap><input id="get_pwrc-' + count[j].toLowerCase() + '" type="number" value="' + count[j+1] + '"><input type="submit" value=">" onclick="setParam(\'get_PwrC(' + count[j++] + ')\');"></td></tr>';
												}
												document.getElementById(valueid).innerHTML = content;
											}
										} else if(values[0] == 'get_tblPDS') {
											var content = "", upsens = "";
											var trows = values[1].split('|');
											var elem = document.getElementById("get_listdsr");
											for(var j = 0; j < trows.length - 1; j++) {
												var tflds = trows[j].split(';');
												upsens += 'get_Opt(DSO' + j + '),';
												content += '<tr><td><select id="get_opt-dsd' + j + '" onchange="setParam(\'get_Opt(DSD' + j + ')\')">' + elem.innerHTML.replace('>' + tflds[0] + '<', ' selected>' + tflds[0] + '<') + '</select></td><td>' + tflds[1] + '</td>' +
														'<td id="get_opt-dst' + j + '">' + tflds[2] + '</td>' +
														'<td nowrap><input id="get_opt-dss' + j + '" type="text" size="6" value="' + tflds[3] + '"> <input type="submit" value=">" onclick="setDS(\'S\',' + j + ')"></td>' +
														'<td nowrap><input id="get_opt-dse' + j + '" type="text" size="6" value="' + tflds[4] + '"> <input type="submit" value=">" onclick="setDS(\'E\',' + j + ')"></td>' +
														'<td id="get_opt-dso' + j + '" nowrap></td></tr>';
											}
											document.getElementById(valueid).innerHTML = content;
											updateParam(upsens);
										} else {
											var content = values[1].replace(/</g, "&lt;").replace(/\|$/g, "").replace(/\|/g, "</td><td>").replace(/\n/g, "</td></tr><tr><td>");
											var element = document.getElementById(valueid);
											if(element) element.innerHTML = '<td>' + content + '</td>';
										}
									} else {
										element = document.getElementById(values[0]);
										if(element) element.innerHTML = "";
									}
								} else if(type == 'values') {
									var valuevar = values[1].toLowerCase().replace(/[^\w\d]/g, "");
									if(valueid == "get_opt-time_chart") window.time_chart = valuevar;
									element = document.getElementById(valueid + "3");
									if(element) {
										if(element.nodeName == "DIV") {
											if(valuevar == '0') element.style = "display:none"; else element.style = "display:default";
										} else {
											element.value = values[1];
											element.innerHTML = values[1];
										}
									}
									element = document.getElementById(valueid);
									if(element) {
										if(element.nodeName == "DIV") { // css visualization
											if(valuevar == '0') element.style = "display:none"; else element.style = "display:default";
										} else if(element.className == "charsw") {
											element.innerHTML = element.title.substr(valuevar,1);
										} else if(values[1].match(/^E-?\d|^ERR/)) {
											if(element.getAttribute("type") == "submit") alert("Ошибка " + values[1]);
											else element.placeholder = values[1];
										} else if(element != document.activeElement) {
											element.innerHTML = values[1];
											element.value = element.type == "number" ? values[1].replace(/[^\-\d.,]|^\-+$/g, "") : values[1];
										}
									}
									if((element = document.getElementById(valueid + "-div1000"))) {
										if(isNaN(Number(values[1]))) element.placeholder = values[1]; else element.value = (Number(values[1])/1000).toFixed(3);
									}
								} else if(type == 'is') {
									if(values[1] == 0 || values[1].substring(0,1) == 'E') {
										if((element = document.getElementById(valueid))) element.className = "inactive";
										if((element = document.getElementById(valueid))) {
											element = element.getElementsByTagName('select');
											if(element[0]) element[0].disabled = true;
										}
									}
								} else if(type == 'tbv') {
									var element2 = document.getElementById(valueid.replace("val", "err"));
									if(values[1].match(/^E-?\d|^ERR/)) {
										if(element2) {
											if(values[1] == "E-29") element2.innerHTML = "ERR ID";
											else if(values[1] == "E-30") element2.innerHTML = "ERR Func";
											else if(values[1] == "E-31") element2.innerHTML = "ERR Timeout";
											else if(values[1] == "E-32") element2.innerHTML = "ERR CRC";
											else element2.innerHTML = values[1];
										}
									} else {
										if(element2) element2.innerHTML = "OK";
										if((element = document.getElementById(valueid))) {
											var val = values[1];
											if(element.className.substring(0,3) == "Div") val = Number(val) / Number(element.className.substring(3));
											if(element.tagName == "INPUT") element.value = val; else element.innerHTML = val;
											element2 = document.getElementById(valueid.replace("val", "hex"));
											if(element2) element2.value = "0x" + Number(values[1]).toString(16).toUpperCase();
										}
									}
								} else if(values[0] == "USR") {
									if(values[1] != "0") {
										var elements = document.getElementsByName("USR");
										while(elements.length > 0) elements[0].remove();
										if((element = document.getElementById("mlogin"))) element.hidden = false;
										if((element = document.getElementById("MService"))) element.innerHTML = '<a href="system.html"><i class="menu-icon menu-icon-service"></i>Сервис</a>';
									}
								} else if(type == "bar") {
									var elval = Number(values[1]);
									if(elval > 100) elval = 100;
									var mx = element.style["max-height"];
									var hgth = Number(mx.replace("px", "")) * elval / 100;
									var btm = Number(element.getAttribute("name").substr(4)) - hgth + 5;
									element.style = "max-height:" + mx + ";margin-top:" + btm +"px;height:" + hgth + "px";
									element = document.getElementById(valueid + "-TT");
									if(element) element.dataset.tooltip = values[1] + "%";
								} else if(values[0] == "get_uptime") {
									if((element = document.getElementById("get_uptime"))) element.innerHTML = values[1];
									if((element = document.getElementById("get_uptime2"))) element.innerHTML = values[1];
								} else if(values[0] == "test_Mail") {
									setTimeout(loadParam('get_Message(scan_MAIL)'), 1000);
								} else if(values[0] == "test_SMS") {
									setTimeout(loadParam('get_Message(scan_SMS)'), 1000);
								} else if(values[0].indexOf("set_SAVE")==0) {
									if(values[1] >= 0) {
										if(values[0].match(/STATS$/)) alert("Статистика сохранена!");
										else if(values[0].match(/UPD$/)) alert("Успешно!");
										else alert("Настройки сохранены, записано " + values[1] + " байт");
									} else alert("Ошибка, код: " + values[1]);
								} else if(values[0].substr(0, 6) == "RESET_" || values[0] == "set_updateNet") {
									if(values[1] != "") alert(values[1]);
								} else {
									if((element = document.getElementById(valueid))) {
										if(element.nodeName == "DIV") {
											if(values[1] == '0') element.style = "display:none"; else element.style = "display:default";
										} else {
											element.value = values[1];
											element.innerHTML = values[1];
										    if(values[0] == "get_MODE") element.style = values[1] == "В работе" ? "color:green" : "color:red";
										    if(values[0] == "get_MODED") element.style = values[1] == "Ok" ? "color:black" : "color:red";
										}
									}
									if((element = document.getElementById(valueid + "2"))) {
										element.value = values[1];
										element.innerHTML = values[1];
									}
								}
							}
						}
						if(typeof window["loadParam_after"] == 'function') window["loadParam_after"](paramid);
					} // end: if (request.responseText != null)
				} // end: if (request.responseText != null)
//			} else if(request.status == 0) { return; }
			} else if(noretry != true) {
				if(request.status == 401 && location.protocol == "file:") NeedLogin = 1;
				console.log("request.status: " + request.status + " retry load...");
				check_ready = 1;
				setTimeout(function() {
					loadParam(paramid);
				}, urltimeout);
			}
			autoheight(); // update height
		}
	}
}

function dhcp(dcb) {
	var fl = document.getElementById(dcb).checked;
	document.getElementById('get_net-ip').disabled = fl;
	document.getElementById('get_net-subnet').disabled = fl;
	document.getElementById('get_net-gateway').disabled = fl;
	document.getElementById('get_net-dns').disabled = fl;
	document.getElementById('get_net-ip2').disabled = fl;
	document.getElementById('get_net-subnet2').disabled = fl;
	document.getElementById('get_net-gateway2').disabled = fl;
	document.getElementById('get_net-dns2').disabled = fl;
}

function validip(valip) {
	var valid = /\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\b/.test(document.getElementById(valip).value);
	if(!valid) alert('Сетевые настройки введены неверно!');
	return valid;
}

function validmac(valimac) {
	var valid = /^[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}$/.test(document.getElementById(valimac).value);
	if(!valid) alert('Аппаратный mac адрес введен неверно!');
	return valid;
}

function unique(arr) {
	var result = [];

	nextInput:
		for(var i = 0; i < arr.length; i++) {
			var str = arr[i]; // для каждого элемента
			for(var j = 0; j < result.length; j++) { // ищем, был ли он уже?
				if(result[j] == str) continue nextInput; // если да, то следующий
			}
			result.push(str);
		}

	return result;
}

function toggletable(elem) {
	el = document.getElementById(elem);
	el.style.display = (el.style.display == 'none') ? 'table' : 'none'
}

function toggleclass(elem, onoff) {
	var elems = document.getElementsByClassName(elem);
	for(var i = 0; i < elems.length; i++) {
		el = elems[i];
		if(onoff) el.style.display = 'block';
		if(!onoff) el.style.display = 'none';
	}
}

var upload_error = false;

function upload(file) {
	var xhr = new XMLHttpRequest();
//	xhr.upload.onprogress = function(event) { console.log(event.loaded + ' / ' + event.total); }
//	xhr.onload = xhr.onerror = function() {
//		if(this.status == 200) { console.log("success"); } else { console.log("error " + this.status); }
//	};
	xhr.open("POST", urlcontrol + (NeedLogin == 2 ? "/&&" + LPString : ""), false);
	xhr.setRequestHeader('Title', file.settings ? "*SETTINGS*" : encodeURIComponent(file.name));
	xhr.onreadystatechange = function() {
		if(this.readyState != 4) return;
		if(xhr.status == 200) {
			if(xhr.responseText != null && xhr.responseText != "") {
				upload_error = true;
				alert(xhr.responseText);
			}
		}
	}
	xhr.send(file);
}

function autoheight() {
/*	var max_col_height = Math.max(
		document.body.scrollHeight, document.documentElement.scrollHeight,
		document.body.offsetHeight, document.documentElement.offsetHeight,
		document.body.clientHeight, document.documentElement.clientHeight
	);*/
	var max_col_height = window.innerHeight;
	var columns = document.body.children;
	for(var i = columns.length - 1; i >= 0; i--) { // прокручиваем каждую колонку в цикле
		if(columns[i].offsetHeight > max_col_height) {
			max_col_height = columns[i].offsetHeight; // устанавливаем новое значение максимальной высоты
		}
	}
	for(var i = columns.length - 1; i >= 0; i--) {
		columns[i].style.minHeight = max_col_height; // устанавливаем высоту каждой колонки равной максимальной
	}
	document.body.style.minWidth = Math.max(document.body.clientWidth, document.body.scrollWidth);
}

function calcacp() {
	var a1 = document.getElementById("a1").value;
	var a2 = document.getElementById("a2").value;
	var p1 = document.getElementById("p1").value;
	var p2 = document.getElementById("p2").value;
	kk2 = (p2 - p1) * 100 / (a2 - a1);
	kk1 = p1 * 100 - kk2 * a1;
	document.getElementById("k1").innerHTML = Math.abs(Math.round(kk1));
	document.getElementById("k2").innerHTML = Math.abs(kk2.toFixed(3));
}

function setKanalog() {
	var k1 = document.getElementById("k1").innerHTML;
	var k2 = document.getElementById("k2").innerHTML;
	var sens = document.getElementById("get_listadc").selectedOptions[0].text;
	if(sens == "--") return;
	var elem = document.getElementById("get_tadc-" + sens.toLowerCase());
	if(elem) {
		elem.value = k2;
		elem.innerHTML = k2;
	}
	elem = document.getElementById("get_zadc-" + sens.toLowerCase());
	if(elem) {
		elem.value = k1;
		elem.innerHTML = k1;
	}
	setParam('get_zADC(' + sens + ')');
	setParam('get_tADC(' + sens + ')');
}

function updateParam(paramids) {
	setInterval(function() { loadParam(paramids) }, urlupdate);
	loadParam(paramids);
}

function updateParamPeriod(paramids, period) {
	setInterval(function() { loadParam(paramids) }, period);
	loadParam(paramids);
}

function updatelog() {
	$('#textarea').load(urlcontrol + '/journal.txt', function() {
		document.getElementById("textarea").scrollTop = document.getElementById("textarea").scrollHeight;
	});
}

function getCookie(name) {
	var matches = document.cookie.match(new RegExp("(?:^|; )" + name.replace(/([\.$?*|{}\(\)\[\]\\\/\+^])/g, '\\$1') + "=([^;]*)"));
	return matches ? decodeURIComponent(matches[1]) : undefined;
}
function setCookie(name, value) {
	document.cookie = name + '="' + escape(value) + '";expires=' + (new Date(2100, 1, 1)).toUTCString();
}