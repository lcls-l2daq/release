ifneq ($(findstring linux,$(tgt_arch)),)
qtincdir  := qt/include
qtlibdir  := qt/QtGui qt/QtCore
qtslibdir :=
else
ifneq ($(findstring rhel6,$(tgt_arch)),)
qtincdir  := 
qtlibdir  := 
qtslibdir := $(USRLIBDIR)/QtGui $(USRLIBDIR)/QtCore
endif
endif

ifneq ($(qtincdir),)
# qwt includes qt headers without package prefix!
qwtincs := $(qtincdir)/Qt
qwtincs += $(qtincdir)/Qt3Support
qwtincs += $(qtincdir)/QtAssistant
qwtincs += $(qtincdir)/QtCore
qwtincs += $(qtincdir)/QtDesigner
qwtincs += $(qtincdir)/QtGui
qwtincs += $(qtincdir)/QtNetwork
qwtincs += $(qtincdir)/QtOpenGL
qwtincs += $(qtincdir)/QtScript
qwtincs += $(qtincdir)/QtSql
qwtincs += $(qtincdir)/QtSvg
qwtincs += $(qtincdir)/QtTest
qwtincs += $(qtincdir)/QtUiTools
qwtincs += $(qtincdir)/QtXml
qwtsinc :=
else
qwtincs :=
qwtsinc := /usr/include/Qt
qwtsinc += /usr/include/Qt3Support
qwtsinc += /usr/include/QtAssistant
qwtsinc += /usr/include/QtCore
qwtsinc += /usr/include/QtDesigner
qwtsinc += /usr/include/QtGui
qwtsinc += /usr/include/QtNetwork
qwtsinc += /usr/include/QtOpenGL
qwtsinc += /usr/include/QtScript
qwtsinc += /usr/include/QtSql
qwtsinc += /usr/include/QtSvg
qwtsinc += /usr/include/QtTest
qwtsinc += /usr/include/QtUiTools
qwtsinc += /usr/include/QtXml
endif
