![Box2D Logo](https://box2d.org/images/logo.svg)

# Build Status
[![Build Status](https://github.com/erincatto/box2c/actions/workflows/build.yml/badge.svg)](https://github.com/erincatto/box2c/actions)

# Box2D v3.0 Notes
This repository is pre-alpha. Everything is a work in progress.

# Giving Feedback
Please visit the discussions tab or start a chat on discord.

# Box2D 
Box2D is a 2D physics engine for games.

## Contributing
Please do not submit pull requests with new features or core library changes. Instead, please file an issue first for discussion. For bugs, I prefer detailed bug reports over pull requests.

## Documentation
- [reddit](https://www.reddit.com/r/box2d/)
- [Discord](https://discord.gg/NKYgCBP)

## License
Box2D is developed by Erin Catto, and uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

## Sponsorship
Support development of Box2D through [Github Sponsors](https://github.com/sponsors/erincatto)

## Building on clang in windows
cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 -T ClangCL
https://clang.llvm.org/docs/UsersManual.html#clang-cl
