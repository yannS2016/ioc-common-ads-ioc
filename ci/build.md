# Building the CI image

The CI image is the EPICS environment used to build and test this IOC and the
driver modules that depend on it (Rocky 9, EPICS base, the module dependencies,
and test tools). `.github/workflows/docker.yml` builds and publishes it on every
push to `master` that touches `ci/Dockerfile`, `RELEASE_SITE`, or
`configure/RELEASE`. You can build the same image on your own host.

## Prerequisites

- Docker with BuildKit (Docker 23+ enables it by default).
- Network access to github.com. The build clones EPICS base and the modules
  from `slac-epics` and `pcdshub`.
- Around 10 GB of free disk and some patience. The build compiles EPICS base and
  about a dozen modules.

## Versions come from the repo

The image does not hardcode module versions. The Dockerfile reads them as
build-args:

- each `*_MODULE_VERSION` from `configure/RELEASE` (asyn, motor, twincat-ads, and
  the rest),
- `BASE_MODULE_VERSION` from `RELEASE_SITE` (EPICS base).

The `ARG` defaults in `ci/Dockerfile` mirror those files, so a plain build works.
Passing the derived args reproduces what CI does and catches any default drift.

## Build

Run from the repo root, which is the build context:

```bash
# Derive the same build-args the workflow passes.
args=$(grep -hE '^[A-Z_]+_MODULE_VERSION[[:space:]]*=' configure/RELEASE RELEASE_SITE \
  | sed -E 's/[[:space:]]*=[[:space:]]*/=/; s/[[:space:]]+$//; s/^/--build-arg /' \
  | tr '\n' ' ')

docker build -f ci/Dockerfile $args -t ioc-ads-ci:test .
```

A plain build uses the `ARG` defaults instead:

```bash
docker build -f ci/Dockerfile -t ioc-ads-ci:test .
```

## Verify

Check the environment and that the build-args flowed through:

```bash
docker run --rm ioc-ads-ci:test bash -lc \
  'echo "$EPICS_BASE"; echo "ADS=$ADS_MODULE_VERSION"; ls "$EPICS_BASE/lib/$EPICS_HOST_ARCH" | head'
ADS=R2.0.0-1.1.1
libCap5.so
libCom.a
libCom.so
libCom.so.3.17.6
libca.a
libca.so
libca.so.4.13.5
libdbCore.a
libdbCore.so
libdbCore.so.3.17.0
```

`ADS` should read the version pinned in `configure/RELEASE`.

## What the image contains

- Rocky 9 base, EPICS base at `/cds/group/pcds/epics/base/<version>`.
- Modules under `/cds/group/pcds/epics/<base>/modules/`: asyn, calc, seq, sscan,
  autosave, caPutLog, iocAdmin, motor, ethercatmc, twincat-ads, streamdevice.
- This repo's IOC, built at `/cds/group/pcds/epics/ioc/common/ads-ioc/<version>`.
- Test tools: gtest, cppcheck, clang-format, pyads.

## Published image

CI tags and pushes:

```
ghcr.io/<owner>/ioc-common-ads-ioc/ci:latest
ghcr.io/<owner>/ioc-common-ads-ioc/ci:<commit-sha>
```

`<owner>` is the repo owner, lowercased, so the upstream image is
`ghcr.io/pcdshub/ioc-common-ads-ioc/ci:latest`.
