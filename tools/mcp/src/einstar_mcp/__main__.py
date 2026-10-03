"""einstar-mcp over stdio (registered for Claude Code in the repository's .mcp.json)."""

from __future__ import annotations

import anyio
from mcp.server.stdio import stdio_server

from .server import make_server


def main() -> None:
    server, impl = make_server()

    async def run() -> None:
        async with stdio_server() as (read, write):
            await server.run(read, write, server.create_initialization_options())

    try:
        anyio.run(run)
    finally:
        impl.supervisor.quit_all()


if __name__ == "__main__":
    main()
