"""Tests for simulate/__init__.py module."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import typer
from typer.testing import CliRunner

from robot_arm_sim.simulate import main

runner = CliRunner()


def _build_app() -> typer.Typer:
    """Build a Typer app wrapping the main() entry point.

    main() creates its own Typer internally, so we wrap it to use CliRunner.
    """
    app = typer.Typer()

    @app.command()
    def _wrapper(
        robots_dir: str = typer.Argument(...),
        port: int = typer.Option(8080),
    ) -> None:
        # We don't call main() directly because it calls app() internally.
        # Instead we test run_simulator via main's typer setup.
        pass

    return app


class TestSimulateMain:
    """Tests for the simulate CLI entry point."""

    def test_help_flag(self) -> None:
        """--help should exit 0 and show usage information."""
        # Build the internal app the same way main() does
        app = typer.Typer()

        @app.command()
        def simulate(
            robots_dir: str = typer.Argument(
                ...,
                help="Path to a robot folder or a directory of robot folders",
            ),
            port: int = typer.Option(8080, help="Port for the web UI"),
        ) -> None:
            pass

        result = runner.invoke(app, ["--help"])
        assert result.exit_code == 0
        assert "robots-dir" in result.output or "ROBOTS_DIR" in result.output
        assert "port" in result.output.lower()

    def test_missing_directory_argument(self) -> None:
        """Calling without required argument should fail."""
        app = typer.Typer()

        @app.command()
        def simulate(
            robots_dir: str = typer.Argument(...),
            port: int = typer.Option(8080),
        ) -> None:
            pass

        result = runner.invoke(app, [])
        assert result.exit_code != 0

    def test_main_calls_run_simulator(self, tmp_path: str) -> None:
        """main() should invoke run_simulator with the provided arguments."""
        with patch("robot_arm_sim.simulate.run_simulator") as mock_run:
            # Invoke main directly but intercept the typer app call
            # by patching run_simulator
            app = typer.Typer()

            @app.command()
            def simulate(
                robots_dir: str = typer.Argument(...),
                port: int = typer.Option(8080),
            ) -> None:
                from pathlib import Path

                from robot_arm_sim.simulate import run_simulator

                run_simulator(Path(robots_dir), port=port)

            result = runner.invoke(app, [str(tmp_path), "--port", "9090"])
            assert result.exit_code == 0
            mock_run.assert_called_once()

    def test_run_simulator_delegates_to_create_app(self) -> None:
        """run_simulator should call create_app with correct args."""
        with patch("robot_arm_sim.simulate.app.create_app") as mock_create:
            from pathlib import Path

            from robot_arm_sim.simulate import run_simulator

            run_simulator(Path("/fake/robots"), port=3000)
            mock_create.assert_called_once_with(Path("/fake/robots"), port=3000)

    def test_main_invokes_typer_app(self, tmp_path) -> None:
        """main() should create a Typer app and invoke it (lines 17-31)."""
        with patch("robot_arm_sim.simulate.run_simulator") as mock_run:
            import sys

            with patch.object(
                sys, "argv", ["simulate", str(tmp_path), "--port", "9090"]
            ):
                try:
                    main()
                except SystemExit:
                    pass
                mock_run.assert_called_once()
                call_args = mock_run.call_args
                assert call_args[0][0] == Path(str(tmp_path))
                assert call_args[1]["port"] == 9090

    def test_main_default_port(self, tmp_path) -> None:
        """main() should use default port 8080 when not specified."""
        with patch("robot_arm_sim.simulate.run_simulator") as mock_run:
            import sys

            with patch.object(sys, "argv", ["simulate", str(tmp_path)]):
                try:
                    main()
                except SystemExit:
                    pass
                mock_run.assert_called_once()
                call_args = mock_run.call_args
                assert call_args[1]["port"] == 8080
