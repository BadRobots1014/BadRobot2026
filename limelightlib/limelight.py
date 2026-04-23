import ipaddress
import json
from pathlib import Path
import socket
import threading
from typing import Any

import ifaddr
from ntcore import NetworkTableInstance
import requests
import websocket
import wpilib

REQUEST_TIMEOUT: float = 2.0


def broadcast_message(message: str, port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(message.encode(), ("255.255.255.255", port))


def broadcast_on_all_interfaces(message: str, port: int, debug: bool) -> None:
    networks: list[tuple[str, str, ipaddress.IPv4Address]] = []

    for adapter in ifaddr.get_adapters():
        for ip in adapter.ips:
            if isinstance(ip.ip, str):
                net = ipaddress.ip_network(f"{ip.ip}/{ip.network_prefix}", strict=False)
                networks.append((adapter.name, ip.ip, net.broadcast_address))

    for name, ip, broadcast in networks:
        if debug:
            print(f"Adapter: {name}, IP: {ip}, Broadcast: {broadcast}")

    for _, _, broadcast in networks:
        try:
            with socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
            ) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.sendto(message.encode(), (str(broadcast), port))
        except Exception as e:
            print(f"Failed to broadcast on {broadcast}: {e}")


def listen_for_responses(port: int, timeout: float = 1) -> list[str]:
    discovered_devices: list[str] = []

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.bind(("", port))
        sock.settimeout(timeout)

        try:
            while True:
                _, addr = sock.recvfrom(1024)  # fixed RUF059
                discovered_devices.append(addr[0])
        except TimeoutError:
            pass

    return discovered_devices


# Landon and Michael tested and this broke the whole robot

# def discover_limelights(
#     broadcast_port: int = 5809,
#     listen_port: int = 5809,
#     timeout: float = 2,
#     debug: bool = False,
# ) -> list[str]:
#     broadcast_on_all_interfaces("LLPhoneHome", broadcast_port, debug)
#     return listen_for_responses(listen_port, timeout)


class Limelight:
    def __init__(self, address: str) -> None:
        self.base_url: str = f"http://{address}:5807"
        self.ws_url: str = f"ws://{address}:5806"

        self.latest_results: dict[str, Any] | None = None
        self.ws: websocket.WebSocketApp | None = None
        self.ws_thread: threading.Thread | None = None

        # use network tables instead of rest for mt2 and orientation because its faster and harder to get a timestamp
        # on rest api

        self.nt_inst = NetworkTableInstance.getDefault()
        self.nt_table = self.nt_inst.getTable("limelight " + address)

        self.robot_pose_mt2_topic = self.nt_table.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        )
        self.robot_pose_mt2_sub = self.robot_pose_mt2_topic.subscribe([])

        self.stddevs_sub = self.nt_table.getDoubleArrayTopic("stddevs").subscribe(
            [0] * 12
        )

        self.tv_sub = self.nt_table.getIntegerTopic("tv").subscribe(0)

        self.orientation_set_pub = self.nt_table.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()

    # ---- internal helpers ----
    def _get(self, endpoint: str, **kwargs: Any) -> requests.Response:
        try:
            return requests.get(
                f"{self.base_url}{endpoint}", timeout=REQUEST_TIMEOUT, **kwargs
            )
        except requests.exceptions.Timeout:
            print(f"Timeout on get {endpoint}")
            return requests.Response()

    def _post(self, endpoint: str, **kwargs: Any) -> requests.Response:
        if not wpilib.RobotBase.isReal():
            return requests.Response()
        try:
            return requests.post(
                f"{self.base_url}{endpoint}", timeout=REQUEST_TIMEOUT, **kwargs
            )
        except requests.exceptions.Timeout:
            print(f"Timeout on post {endpoint}")
            return requests.Response()

    def _delete(self, endpoint: str, **kwargs: Any) -> requests.Response:
        try:
            return requests.delete(
                f"{self.base_url}{endpoint}", timeout=REQUEST_TIMEOUT, **kwargs
            )
        except requests.exceptions.Timeout:
            print(f"Timeout on delete {endpoint}")
            return requests.Response()

    # ---- basic endpoints ----
    def get_results(self) -> dict[str, Any]:
        return self._get("/results").json()

    def get_status(self) -> dict[str, Any] | None:
        response = self._get("/status")
        return response.json() if response.ok else None

    def hw_report(self) -> dict[str, Any]:
        return self._get("/hwreport").json()

    def reload_pipeline(self) -> requests.Response:
        return self._post("/reload-pipeline")

    # ---- pipelines ----
    def get_pipeline_default(self) -> dict[str, Any] | None:
        response = self._get("/pipeline-default")
        return response.json() if response.ok else None

    def get_pipeline_atindex(self, index: int) -> dict[str, Any] | None:
        response = self._get("/pipeline-atindex", params={"index": index})
        return response.json() if response.ok else None

    def pipeline_switch(self, index: int) -> requests.Response:
        return self._post("/pipeline-switch", params={"index": index})

    def get_snapscript_names(self) -> list[str]:
        return self._get("/getsnapscriptnames").json()

    # ---- snapshots ----
    def capture_snapshot(self, snapname: str = "") -> requests.Response:
        return self._post("/capture-snapshot", params={"snapname": snapname})

    def upload_snapshot(self, snapname: str, image_path: str) -> requests.Response:
        with Path(image_path).open("rb") as image_file:
            return self._post(
                "/upload-snapshot",
                params={"snapname": snapname},
                files={"file": image_file},
            )

    def snapshot_manifest(self) -> dict[str, Any]:
        return self._get("/snapshotmanifest").json()

    def delete_snapshots(self) -> requests.Response:
        return self._delete("/delete-snapshots")

    def delete_snapshot(self, snapname: str) -> requests.Response:
        return self._delete("/delete-snapshot", params={"snapname": snapname})

    # ---- pipeline updates ----
    def update_pipeline(
        self, profile_json: str, flush: int | None = None
    ) -> requests.Response:
        params: dict[str, Any] = {}
        if flush is not None:
            params["flush"] = flush

        response = self._post(
            "/update-pipeline",
            headers={"Content-Type": "application/json"},
            params=params,
            data=profile_json,
        )

        if response.status_code == 400:
            try:
                print("Error:", response.json())
            except ValueError:
                print("Error:", response.text)

        return response

    def update_python_inputs(self, inputs: dict[str, Any]) -> requests.Response:
        return self._post(
            "/update-pythoninputs",
            headers={"Content-Type": "application/json"},
            data=json.dumps(inputs),
        )

    # Set Robot Orientation and angular velocities in degrees and degrees per second
    def robot_orientation_set(
        self,
        yaw: float,
    ) -> None:
        self.orientation_set_pub.set([yaw, 0, 0, 0, 0, 0])

    def update_throttle(self, skip_frames: int) -> requests.Response:
        return self._post(
            "/update-throttle",
            headers="Content-Type: application/json",
            data=json.dumps(skip_frames),
        )

    def update_imumode(self, imumode: int) -> requests.Response:
        return self._post(
            "/update-imumode",
            headers="Content-Type: application/json",
            data=json.dumps(imumode),
        )

    # ---- uploads ----
    def upload_pipeline(
        self, profile_json: str, index: int | None = None
    ) -> requests.Response:
        params = {"index": index} if index is not None else {}
        return self._post(
            "/upload-pipeline",
            headers={"Content-Type": "application/json"},
            params=params,
            data=profile_json,
        )

    def upload_fieldmap(
        self, fieldmap_json: str, index: int | None = None
    ) -> requests.Response:
        params = {"index": index} if index is not None else {}
        return self._post(
            "/upload-fieldmap",
            headers={"Content-Type": "application/json"},
            params=params,
            data=fieldmap_json,
        )

    def upload_python(
        self, pythonstring: str, index: int | None = None
    ) -> requests.Response:
        params = {"index": index} if index is not None else {}
        return self._post(
            "/upload-python",
            headers={"Content-Type": "text/plain"},
            params=params,
            data=pythonstring,
        )

    def upload_neural_network(
        self, nn_type: str, file_path: str, index: int | None = None
    ) -> requests.Response:
        params = {"type": nn_type, **({"index": index} if index is not None else {})}
        with Path(file_path).open("rb") as f:
            return self._post(
                "/upload-nn",
                params=params,
                headers={"Content-Type": "application/octet-stream"},
                data=f.read(),
            )

    def upload_neural_network_labels(
        self, nn_type: str, file_path: str, index: int | None = None
    ) -> requests.Response:
        params = {"type": nn_type, **({"index": index} if index is not None else {})}
        with Path(file_path).open("rb") as f:
            return self._post(
                "/upload-nnlabels",
                params=params,
                headers={"Content-Type": "text/plain"},
                data=f.read(),
            )

    # ---- calibration ----
    def cal_default(self) -> dict[str, Any]:
        return self._get("/cal-default").json()

    def cal_file(self) -> dict[str, Any]:
        return self._get("/cal-file").json()

    def cal_eeprom(self) -> dict[str, Any]:
        return self._get("/cal-eeprom").json()

    def cal_latest(self) -> dict[str, Any]:
        return self._get("/cal-latest").json()

    def update_cal_eeprom(self, cal_data: str) -> requests.Response:
        return self._post("/cal-eeprom", data=cal_data)

    def update_cal_file(self, cal_data: str) -> requests.Response:
        return self._post("/cal-file", data=cal_data)

    def delete_cal_latest(self) -> requests.Response:
        return self._delete("/cal-latest")

    def delete_cal_eeprom(self) -> requests.Response:
        return self._delete("/cal-eeprom")

    def delete_cal_file(self) -> requests.Response:
        return self._delete("/cal-file")

    # ---- convenience ----
    def get_name(self) -> str | None:
        status = self.get_status()
        return status.get("name") if status else None

    def get_temp(self) -> float | None:
        status = self.get_status()
        return status.get("temp") if status else None

    def get_fps(self) -> float | None:
        status = self.get_status()
        return status.get("fps") if status else None

    # ---- websocket ----
    def enable_websocket(self) -> None:
        def on_message(_ws: websocket.WebSocketApp, message: str) -> None:
            self.latest_results = json.loads(message)

        def run() -> None:
            self.ws = websocket.WebSocketApp(self.ws_url, on_message=on_message)
            self.ws.run_forever()

        self.ws_thread = threading.Thread(target=run)
        self.ws_thread.start()

    def disable_websocket(self) -> None:
        if self.ws:
            self.ws.close()
        if self.ws_thread:
            self.ws_thread.join()

    def get_latest_results(self) -> dict[str, Any] | None:
        return self.latest_results
