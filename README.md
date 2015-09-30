<<<<<<< HEAD
```
```
#
``` mycomponent
```
Intro to component here


## Configuration parameters
As any other component,
``` *mycomponent* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <mycomponent 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```mycomponent ```

    --Ice.Config=config
=======
# Robotica2015
>>>>>>> 1ed976970d9ab1f03f05454f291e35870fb49d05
