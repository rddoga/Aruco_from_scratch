#ifndef VERSION_H
#define VERSION_H

#define TOSTRING_HELPER(x) #x
//We need this additional preprocessor indirection because otherwise it won't work!
#define TOSTRING(x) TOSTRING_HELPER(x)

#define VIMBAVIEWER_VERSION_MAJOR    2
#define VIMBAVIEWER_VERSION_MINOR    5
#define VIMBAVIEWER_VERSION_PATCH    0
#define VIMBAVIEWER_VERSION          (TOSTRING(VIMBAVIEWER_VERSION_MAJOR) "." TOSTRING(VIMBAVIEWER_VERSION_MINOR) "." TOSTRING(VIMBAVIEWER_VERSION_PATCH))

#define VIMBAVIEWER_COPYRIGHT_STRING "Copyright (c) Allied Vision Technologies 2012 - 2021"

#endif //VERSION_H
