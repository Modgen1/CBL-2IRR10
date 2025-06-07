from http.client import HTTPException
from math import sqrt
import requests


def calculate_distance(x1: int, x2: int, y1: int, y2: int) -> float:
    return sqrt(((x1 - x2) ** 2 + (y1 - y2) ** 2))


class CoordinateCommunicator:
    APIBASE = "https://modgen.nl/api/"

    # bool isDelivering
    # tuple (int, int) pickup
    # tuple (int, int) dropoff
    # int deliveryID
    __coordinates = {1: {}, 2: {}, 3: {}}

    def get_coordinates(self, robotID: int, x: int, y: int) -> tuple[int, int]:
        """
        Gets the coordinates the robot should move to. If there is no current delivery,
        the robot first fetches one.

        :param robotID: The robot ID.
        :param x: The current x coordinate.
        :param y: The current y coordinate.
        :returns: A tuple containing the x and y coordinates to move to.
        """

        data = self.__coordinates[robotID]

        if data.get("isDelivering", False):
            return data["dropoff"]

        pickup: tuple[int, int] | None = data.get("pickup", None)

        # Pickup already retrieved from database
        if pickup is not None:
            return pickup

        try:
            pickup = self.__get_pickup(robotID, x, y)
        except HTTPException as e:
            print("Unable to retrieve pickup from API: ", e)
            return x, y

        return pickup

    def __get_pickup(self, robotID: int, x: int, y: int) -> tuple[int, int]:
        """
        Gets a pickup from the API.

        :param robotID: Robot id
        :param x: Robot's current x-coordinate
        :param y: Robot's current y-coordinate
        :return: Tuple containing the x and y coordinates of the pickup, or the current robot location if there was no suitable pickup
        :rtype: tuple[int, int]
        """
        selected = self.__select_pickup(x, y)

        if selected is None:
            print("No pickup available!")
            return x, y

        # Send taken request to api
        r = requests.post(self.APIBASE + "take", params={"req_id": selected["id"]})
        if r.status_code != 200:
             raise HTTPException("Unable to take delivery!")

        print(f"Robot {robotID} has taken delivery {selected['id']}")

        self.__update_data(robotID, selected)

        return selected["pick_up_x"], selected["pick_up_y"]

    def __update_data(self, robotID: int, data: dict):
        self.__coordinates[robotID]["isDelivering"] = True
        self.__coordinates[robotID]["pickup"] = (data["pick_up_x"], data["pick_up_y"])
        self.__coordinates[robotID]["dropoff"] = (data["drop_off_x"], data["drop_off_y"])
        self.__coordinates[robotID]["deliveryID"] = data["id"]

    def __select_pickup(self, x: int, y: int) -> dict | None:
        """
        Selects the most suitable delivery pickup entry based on availability, priority, and distance
        from the provided coordinates.

        :param x: Robot's current x-coordinate
        :param y: Robot's current y-coordinate
        :return: A dictionary containing the selected pickup entry or None if no suitable entry is found
        :rtype: dict | None
        :raises HTTPException: If the API call to retrieve the list of available pickups fails
        """
        r = requests.get(self.APIBASE + "list")

        if r.status_code != 200:
            raise HTTPException()

        available = r.json()

        selected = None

        # Find delivery
        for entry in available:
            if entry["taken"]:
                continue

            if selected is None:
                selected = entry
                continue

            if entry["priority"] > selected["priority"]:
                selected = entry
                continue

            if entry["priority"] < selected["priority"]:
                continue

            if calculate_distance(entry["pick_up_x"], x, entry["pick_up_y"], y) \
                    < calculate_distance(selected["pick_up_x"], x, selected["pick_up_y"], y):
                selected = entry

        return selected

    def complete_delivery(self, robotID: int, deliveryID: int) -> None:
        """
        Completes a delivery.

        :param robotID: ID of the robot completing the delivery.
        :param deliveryID: ID of the delivery to complete.
        """
        r = requests.post(self.APIBASE + "remove", params={"del_id": deliveryID})

        if r.status_code != 200:
            raise HTTPException("Unable to remove delivery from API!")

        response = r.json()
        if response["deleted_id"] is None:
            raise KeyError(f"There is no delivery with ID {deliveryID}!")

        self.__coordinates[robotID]["isDelivering"] = False
        self.__coordinates[robotID]["pickup"] = None
        self.__coordinates[robotID]["dropoff"] = None
        self.__coordinates[robotID]["deliveryID"] = None

    def get_delivery_id(self, robotID: int) -> int | None:
        return self.__coordinates[robotID].get("deliveryID", None)