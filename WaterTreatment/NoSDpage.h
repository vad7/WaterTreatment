/*
 * Copyright (c) 2020-2022 by Vadim Kulakov vad7@yahoo.com, vad711
  *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 */
// ------------------------------------------------------------------------------------------------------------------------
// Хранение минимальной веб морды в памяти контроллера, используется если отказ сд карты ----------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#ifndef NoSDpage_h
#define NoSDpage_h

// Конвертор http://www.buildmystring.com/default.php
// индексная страница без карты
#define NO_SD_SHOW_SETTINGS
const char index_noSD[] =
		"<html><head><title>Водоподготовка</title><meta charset='utf-8'/><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
		"<script type='text/javascript'>"
		"window.onload = function() { \ndocument.getElementsByTagName('body')[0].innerHTML = document.getElementsByTagName('body')[0].innerHTML.replace(/\\n/g, '<br>'); };"
		"</script></head>\n<body><h1>Водоподготовка</h1><br><br>";

const char index_noSD_end[] = "</body></html>";
#endif
