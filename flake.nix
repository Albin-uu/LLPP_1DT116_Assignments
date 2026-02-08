{
  description = "dev shell for pedsim (converted from shell.nix)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {inherit system;};
    in {
      devShells.default = pkgs.mkShell rec {
        buildInputs = with pkgs; [
          libgcc
          zlib
          gcc
          cmake
          pkg-config
          qt5.qtbase
          qt5.qtmultimedia
          qt5.qtwayland
          qt5.qttools
          libxkbcommon
          libGL
          wayland
          xorg.libXcursor
          xorg.libXrandr
          xorg.libXi
          xorg.libX11
          llvmPackages.openmp
          tinyxml-2
          python313Packages.matplotlib
          python313Packages.numpy
          bear
          just
          gdb
        ];

        LD_LIBRARY_PATH = "${pkgs.stdenv.cc.cc.lib}/lib/:${
          pkgs.lib.makeLibraryPath buildInputs
        }:$LD_LIBRARY_PATH";

        shellHook = ''
          echo "Entering pedsim dev shell"
          echo "Qt5 available: $(pkg-config --exists Qt5Core && echo yes || echo no)"
          echo "CUDA available: $(command -v nvcc >/dev/null && echo yes || echo no)"
          echo "CUDA_AVAILABLE=" > ./demo/config.mk
        '';
      };
    });
}
