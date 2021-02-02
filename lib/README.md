
This directory is intended for project specific (private) libraries.
Mbed will compile them to static libraries and link into executable file.

The source code of each library should be placed in a an own separate directory
("lib/your_library_name/[here are source files]").

For example, see a structure of the following two libraries `Foo` and `Bar`:
```
|--lib
|  |
|  |--Bar
|  |  |--docs
|  |  |--examples
|  |  |--src
|  |     |- Bar.cpp
|  |     |- Bar.h
|  |  |- mbed_library.json (optional, custom build options, etc)
|  |--Foo
|  |  |- Foo.cpp
|  |  |- Foo.h
|  |
|  |- README --> THIS FILE
|
|- mbed_app.json
|--src
|  |- main.cpp
```
and a contents of `src/main.cpp`:
```
#include <Foo.h>
#include <Bar.h>

int main ()
{
  ...
}

```
