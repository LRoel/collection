(tk_global_version_1.cpp) version 1 : 一次整体的GPS数据计算的RT矩阵，不存在局部覆盖
(tk_global_version_2.cpp) version 2 : 一次整体的GPS数据计算的RT矩阵，并且覆盖公司地图西北角
(tk_global_version_3.cpp) version 3 : 换数据一次GPS数据整体计算RT矩阵
(tk_global_version_4.cpp) version 4 : RT_ver4与RT_ver1覆盖融合，西南角为ver1 RT，其他部分为ver4 RT
		    映射结果在公司西侧、西北侧、西南侧、东北侧、南侧较好，在北侧中部、东南部较差
(tk_global_version_6.cpp) version 6 : RT_ver4与RT_ver5覆盖融合，东南角为ver5 RT，其他部分为ver4 RT   ver5为Gnss/Odom融合结果与Amcl计算的RT矩阵
		    映射结果在公司西侧、西北侧、西南侧、东北侧、南侧、东南部较好，在北侧中部较差（正在使用的RT矩阵）((tk_global.cpp) )

