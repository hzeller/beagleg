{
  description = "BeagleG dev shell";

  # nixos-unstable @ 2026-06-26
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/e73de5be04e0eff4190a1432b946d469c794e7b4";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  # Follow the am335x_pru_package git submodule transparently, so users
  # don't need to add ?submodules=1 to every nix-build invocation.
  inputs.self.submodules = true;

  outputs = { self, nixpkgs, flake-utils }:
    # BeagleG is linux-only; expose the flake on every linux system
    # nixpkgs considers reasonable (incl. armv7l-linux for native builds
    # on a BeagleBone itself, aarch64-linux for RPi-class boards, etc.).
    flake-utils.lib.eachSystem
      (nixpkgs.lib.filter (s: nixpkgs.lib.hasSuffix "-linux" s)
        nixpkgs.lib.systems.flakeExposed)
      (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lib = pkgs.lib;
        beagleg = pkgs.callPackage ./nix/beagleg.nix { };

        # Cross-compile targets. Add one line per new architecture; the
        # suffix becomes the flake-output suffix (e.g. machine-control-armv7).
        crossTargets = {
          armv7 = pkgs.pkgsCross.armv7l-hf-multiplatform;
        };

        # Binaries shipped by the package. Each becomes its own flake
        # output with mainProgram set, so `nix run .#gcode2ps` picks the
        # right entrypoint. All outputs share the same underlying build.
        binaries = [ "machine-control" "gcode-print-stats" "gcode2ps" ];
        mkRunnable = drv: name: drv.overrideAttrs (_: {
          meta = drv.meta // { mainProgram = name; };
        });
        named = drv: suffix: lib.listToAttrs (map (name: {
          name = name + suffix;
          value = mkRunnable drv name;
        }) binaries);
        crossPackages = lib.concatMapAttrs
          (suffix: cross: named (cross.callPackage ./nix/beagleg.nix { }) "-${suffix}")
          crossTargets;
      in {
        devShells.default = import ./shell.nix { inherit pkgs; };
        packages = { default = beagleg; }
          // (named beagleg "")
          // crossPackages;
      });
}
