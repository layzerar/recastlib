import os
from distutils.core import setup, Extension
import env

libext_name = '_recast'


def findall(dir=os.curdir):
    """Find all files under 'dir' and return the list of full filenames
    (relative to 'dir').
    """
    all_files = []
    for base, dirs, files in os.walk(dir):
        if base==os.curdir or base.startswith(os.curdir+os.sep):
            base = base[2:]
        if base:
            files = [os.path.join(base, f) for f in files]
        all_files.extend(filter(os.path.isfile, files))
    return all_files


def endswith(string, suffixs=()):
    for suffix in suffixs:
        if string.endswith(suffix):
            return True
    return False


def findcxx(dir=os.curdir, suffixs=('.c', '.cpp', '.cxx', '.cc')):
    cxx_files = []
    for file in findall(dir=dir):
        if not endswith(file, suffixs=suffixs):
            continue
        cxx_files.append(file)
    return cxx_files


def main():
    sources = []
    include_dirs = []
    library_dirs = []
    libraries = []
    extra_compile_args = []
    extra_link_args = []

    include_dirs += [
        'Recast/Detour/Include',
        'Recast/DetourTileCache/Include',
        'Recast/DetourCrowd/Include',
        'Recast/DebugUtils/Include',
        'Recast/Recast/Include',
        'python',
        ]
    libraries += ['boost_python']
    include_dirs += env.boost_include
    library_dirs += env.boost_library_path
    sources += findcxx('Recast/Detour')
    sources += findcxx('python')

    if os.name == "posix":
        include_dirs += [r'/usr/local/include/', r'/usr/include/']
        library_dirs += [r'/usr/local/lib64/', r'/usr/local/lib/', r'/usr/lib64/', r'/usr/lib/']
        extra_compile_args += [
            '-fvisibility-inlines-hidden',
            '-ffunction-sections',
            '-fdata-sections',
        ]
        extra_link_args += [
            '-Wl,--gc-sections',
            '-Wl,--sort-common',
            '-Wl,--sort-section,alignment',
        ]
    elif os.name == "nt":
        extra_compile_args += [
            '/EHsc',
        ]

    libext = Extension(
        name=libext_name,
        sources=sources,
        include_dirs=include_dirs,
        library_dirs=library_dirs,
        libraries=libraries,
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        language='c++',
    )

    setup(
        name=libext_name,
        ext_modules=[libext]
    )


if __name__ == '__main__':
    main()
