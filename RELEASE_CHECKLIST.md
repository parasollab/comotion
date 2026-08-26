# Release Checklist

Run these checks from a fresh clone before publishing a release:

1. Clone with recursive submodules using the public HTTPS URLs.
2. Configure a Release build with apps, examples, and tests enabled.
3. Build the project and run `ctest --output-on-failure`.
4. Run the Python benchmark-runner unit tests.
5. Install to an empty prefix and run the installed library and FCL Panda
   smoke tests.
6. Build the CPack source archive and verify it contains the Panda URDF,
   collision meshes, benchmark tasks, license texts, and notices.
7. Scan the tracked tree and reachable release history for credentials,
   generated results, local absolute paths, and unexpectedly large blobs.
8. Confirm `LICENSE`, `THIRD_PARTY_NOTICES.md`, `LICENSES/`, and `CITATION.cff`
   match the released contents and version.
9. Confirm the release commit uses the intended public author identity.
10. After the release commit passes every check, create and push the annotated
    `v0.1.0` tag from that exact commit.

The GitHub Actions workflow performs the build, test, install, and installed
smoke-test portions on every push and pull request.
