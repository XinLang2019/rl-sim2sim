"""Client disconnects during normal and error responses must stay request-local."""
from types import SimpleNamespace
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
from rlsim_support.catalog import ConfigError
from tools.web.services.http_server import Handler


class HTTPDisconnectTests(unittest.TestCase):
    def handler(self, result=None, error=None):
        handler = Handler.__new__(Handler)
        handler.headers = {'Host': '127.0.0.1:8765'}; handler.path = '/api/bootstrap'
        handler.server = SimpleNamespace(allowed_hosts={'127.0.0.1:8765'}, token='test',
                                         manager=SimpleNamespace(catalog=Mock(return_value=result or {}, side_effect=error)))
        handler.close_connection = False
        return handler

    def test_disconnect_during_error_response_does_not_escape_the_handler(self):
        for error in (ConfigError('missing policy'), ValueError('bad value'), FileNotFoundError('missing file'), FileExistsError('existing file'), RuntimeError('internal failure')):
            for disconnect in (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                with self.subTest(error=type(error), disconnect=disconnect):
                    handler = self.handler(error=error); handler.send = Mock(side_effect=disconnect())
                    handler.dispatch(False)
                    self.assertTrue(handler.close_connection)
                    handler.send.assert_called_once()

    def test_disconnect_during_success_does_not_attempt_an_error_response(self):
        handler = self.handler(); handler.send = Mock(side_effect=BrokenPipeError())
        handler.dispatch(False)
        self.assertTrue(handler.close_connection)
        handler.send.assert_called_once()

    def test_connected_client_still_receives_configuration_errors(self):
        handler = self.handler(error=ConfigError('missing policy')); handler.send = Mock()
        handler.dispatch(False)
        handler.send.assert_called_once_with({'error': 'missing policy'}, 400)
