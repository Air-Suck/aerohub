import asyncio
import threading
import unittest

from fastapi import HTTPException
from pydantic import BaseModel

from aerohub_inference.app import create_app


class Payload(BaseModel):
    value: float


class RuntimeTest(unittest.IsolatedAsyncioTestCase):
    async def test_cancelled_caller_keeps_worker_busy_until_execution_stops(self):
        entered, release = threading.Event(), threading.Event()

        class Predictor:
            metadata = {"model_id": "test", "device": "cpu"}

            def infer(self, payload):
                entered.set()
                release.wait(5)
                return payload

        app = create_app(Predictor, Payload, Payload)
        infer = next(route.endpoint for route in app.routes if route.path == "/infer")
        body = Payload(value=1.)
        async with app.router.lifespan_context(app):
            task = asyncio.create_task(infer(body))
            try:
                for _ in range(100):
                    if entered.is_set():
                        break
                    await asyncio.sleep(0.01)
                self.assertTrue(entered.is_set())
                task.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await task
                with self.assertRaises(HTTPException) as error:
                    await infer(body)
                self.assertEqual(error.exception.status_code, 429)
                self.assertTrue(app.state.busy)
            finally:
                release.set()
                for _ in range(100):
                    if not app.state.busy:
                        break
                    await asyncio.sleep(0.01)
            self.assertFalse(app.state.busy)

    async def test_model_validation_error_releases_slot(self):
        class Predictor:
            metadata = {"model_id": "test"}

            def infer(self, payload):
                raise ValueError("invalid model input")

        app = create_app(Predictor, Payload, Payload)
        infer = next(route.endpoint for route in app.routes if route.path == "/infer")
        async with app.router.lifespan_context(app):
            with self.assertRaises(HTTPException) as error:
                await infer(Payload(value=1.))
            self.assertEqual(error.exception.status_code, 422)
            self.assertFalse(app.state.busy)


if __name__ == "__main__":
    unittest.main()
