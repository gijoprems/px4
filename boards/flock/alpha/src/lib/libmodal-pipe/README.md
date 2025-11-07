# libmodal-pipe

This is a library to support named-pipe communication between a local server and multiple local clients.

To test and demonstrate its function, this includes an example "modal-hello-server" and "modal-hello-client" that can be tried for fun. The server simple sends "hello" to the client once a second. This can be used to test the pipe behavior of multiple clients starting and stopping.

## Dependencies

* [voxl-mavlink](https://gitlab.com/voxl-public/core-libs/voxl-mavlink)  (optional)
* [libmodal-json](https://gitlab.com/voxl-public/core-libs/libmodal_json)


## Build Instructions

1) prerequisite: the voxl-cross docker image >= V1.7

Follow the instructions here to build and install the voxl-cross docker image:

https://gitlab.com/voxl-public/voxl-docker


2) Launch the voxl-cross docker.

```bash
~/git/libmodal-pipe$ voxl-docker -i voxl-cross
voxl-cross:~$
```


3) Install dependencies inside the docker. You must specify both the hardware platform and binary repo section to pull from. CI will use the `dev` binary repo for `dev` branch jobs, otherwise it will select the correct target SDK-release based on tags. When building yourself, you must decide what your intended target is, usually `dev` or `staging`

```bash
voxl-cross:~$ ./install_build_deps.sh qrb5165 dev
```


4) Build for either `qrb5165` or `apq8096`. CI will pass these arguments to the build script based on the job target. The build script in this template will build a multi-arch package with binaries for both 64 and 32 bit.

```bash
voxl-cross:~$ ./build.sh qrb5165
OR
voxl-cross:~$ ./build.sh apq8096
```


5) Make an ipk or deb package while still inside the docker.

```bash
voxl-cross:~$ ./make_package.sh deb
OR
voxl-cross:~$ ./make_package.sh ipk
```

This will make a new package file in your working directory. The name and version number came from the package control file. If you are updating the package version, edit it there.

Optionally add the --timestamp argument to append the current data and time to the package version number in the debian package. This is done automatically by the CI package builder for development and nightly builds, however you can use it yourself if you like.


## Deploy to VOXL

You can now push the ipk or deb package to VOXL and install with dpkg/opkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

The deploy_to_voxl.sh script will query VOXL over adb to see if it has dpkg installed. If it does then then .deb package will be pushed an installed. Otherwise the .ipk package will be installed with opkg.

```bash
(outside of docker)
libmodal-pipe$ ./deploy_to_voxl.sh
```

This deploy script can also push over a network given sshpass is installed and the VOXL uses the default root password.


```bash
(outside of docker)
libmodal-pipe$ ./deploy_to_voxl.sh ssh 192.168.1.123
```

