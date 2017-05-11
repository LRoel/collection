#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import string
from translate import translate_it


code_file = open('test.h','r')
code_content = code_file.read()
p = re.compile(r'((?<=\n)|^)[ \t]*\/\*.*?\*\/\n?|\/\*.*?\*\/|((?<=\n)|^)[ \t]*\/\/[^\n]*\n|\/\/[^\n]*')

upper_string = ''
for m in p.finditer(code_content):
	# 使用Match获得分组信息
	if m.group() == "//www.apache.org/licenses/LICENSE-2.0":
		continue
	elif "namespace" in m.group():
		continue
	elif m.group().upper() == m.group():
		continue
	else:
		tmp_string = m.group().replace("  ",'')
		tmp_string = tmp_string.replace("// ", '')
		tmp_string = tmp_string.replace("\n", ' ')
	# 	elif m.group()[-2] != '.':
	# 	tmp_string = m.group().replace("// ", '')
	# 	tmp_string = tmp_string.replace("\n ", '')
	# 	upper_string += tmp_string
	# else:
		if m.group()[-2] != '.':
			upper_string += tmp_string
		else:
			out_string = upper_string + tmp_string
			upper_string = ''
			print(out_string)
			print(translate_it(out_string))



