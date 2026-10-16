"""Optional stage checkpoints and wall-clock heartbeat; never calls CARLA APIs."""
import json
import threading
import time
from datetime import datetime, timezone
from carla_runtime import save_json


class StageTrace:
    def __init__(self, out, status, enabled=False, heartbeat_seconds=5.0):
        self.out=out;self.status=status;self.enabled=enabled
        self.interval=heartbeat_seconds;self.lock=threading.Lock()
        self.stop=threading.Event();self.thread=None;self.log=None
        self.current=None;self.stage_started=time.monotonic();self.progress={}

    def __enter__(self):
        if self.enabled:
            self.log=(self.out/'stage_trace.jsonl').open('w',encoding='utf-8')
            self.thread=threading.Thread(target=self._heartbeat,daemon=True)
            self.thread.start()
        return self

    def _write(self, event):
        # Caller holds the lock. run_status is written only by the main thread.
        record={'utc':datetime.now(timezone.utc).isoformat(), 'event':event,
                'phase':self.current,
                'stage_wall_elapsed_s':round(time.monotonic()-self.stage_started,3),
                **self.progress}
        self.log.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
        self.log.flush()
        temp=self.out/'stage_progress.json.tmp'
        save_json(temp,record);temp.replace(self.out/'stage_progress.json')
        return record

    def stage(self, phase, description):
        self.status['phase']=phase
        if not self.enabled:return
        # Persist BEFORE entering the next potentially blocking RPC.
        save_json(self.out/'run_status.json',self.status)
        with self.lock:
            self.current=phase;self.stage_started=time.monotonic();self.progress={}
            self._write('stage_begin')
            print(f'[BUOC {phase}] {description}',flush=True)

    def pulse(self, **values):
        if not self.enabled:return
        with self.lock:
            self.progress.update(values)
            self._write('progress')
            text=' | '.join(f'{k}={v}' for k,v in self.progress.items())
            print(f'[TIEN DO {self.current}] {text}',flush=True)
        save_json(self.out/'run_status.json',self.status)

    def _heartbeat(self):
        while not self.stop.wait(self.interval):
            with self.lock:
                if self.current is not None:
                    record=self._write('waiting')
                    print(f'[DANG CHO {self.current}] {record["stage_wall_elapsed_s"]:.1f}s thoi gian thuc; '
                          f'moc gan nhat={self.progress}',flush=True)

    def __exit__(self, exc_type, exc, tb):
        self.stop.set()
        if self.thread is not None:self.thread.join()
        if self.log is not None:
            with self.lock:
                self._write('exception' if exc is not None else 'trace_closed')
                self.log.close()

