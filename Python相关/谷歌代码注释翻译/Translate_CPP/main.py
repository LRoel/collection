#!/usr/bin/env python

import re
import json
import os
import string
from translate import translate_it

code_file = open("test.h.bak",'r')
write_code_file = open("test.h",'w')
code_file_list = code_file.readlines()
p = re.compile(r'((?<=\n)|^)[ \t]*\/\*.*?\*\/\n?|\/\*.*?\*\/|((?<=\n)|^)[ \t]*\/\/[^\n]*\n|\/\/[^\n]*')

upper_string = ''
write_code_list = []
for code_content_ptr in code_file_list:
	m = p.match(code_content_ptr)
	# 使用Match获得分组信息
	if m:
		if m.group() == "//www.apache.org/licenses/LICENSE-2.0" or "namespace" in m.group() or m.group().upper() == m.group():
			write_code_list.append(code_content_ptr)
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
				n = 0
				for i in m.group():
					if i.isspace():
						n += 1
					else:
						break
				out_string_split = out_string.split('.')
				translated_string = ''
				for out_string_split_ptr in out_string_split:
					if out_string_split_ptr != '' or out_string_split_ptr != ' ':
						text = translate_it(out_string_split_ptr)
						if text != '':
							translated_string += translate_it(out_string_split_ptr) + '.'
				print(translated_string)
				x = json.loads('{"foo":"%s"}' % translated_string)
				x0 = x['foo']
				code_content_ptr += " " * n + "/**\n" + " " * n + "* @brief " + x0 + "\n" + " " * n + "*/\n"
				print(code_content_ptr)
			write_code_list.append(code_content_ptr)
	else:
		write_code_list.append(code_content_ptr)
write_code_file.writelines(write_code_list)
write_code_file.close()
code_file.close()
