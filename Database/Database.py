import sqlite3

class Database:
    def __init__(self):
        self.conn = sqlite3.connect('database.db')
        self.cursor = self.conn.cursor()

        self.initialiseDatabase()


    def initialiseDatabase(self):
        query = """ CREATE TABLE IF NOT EXISTS deliveries (
                    id INT NOT NULL PRIMARY KEY,
                    pickupX INT NOT NULL,
                    pickupY INT NOT NULL,
                    dropoffX INT NOT NULL,
                    dropoffY INT NOT NULL)"""
        self.cursor.execute(query)


    def addDelivery(self, idEntry: int, pX: int, pY: int, dX: int, dY: int):
        query = f"""INSERT INTO deliveries (id, pickupX, pickupY, dropoffX, dropoffY)
                    VALUES ({idEntry}, {pX}, {pY}, {dX}, {dY})"""

        try:
            self.cursor.execute(query)
            self.conn.commit()
        except (sqlite3.OperationalError, sqlite3.IntegrityError) as e:
            print(f"Unable to add delivery with ID {idEntry} to database: {e}")
            return

        print(f"Successfully added delivery with ID {idEntry} to database")

    def removeDelivery(self, idEntry):
        query = f"""DELETE FROM deliveries WHERE id = {idEntry}"""
        try:
            self.cursor.execute(query)
            self.conn.commit()
        except sqlite3.OperationalError as e:
            print(f"Unable to remove delivery with ID {idEntry} to database: {e}")
            return

        print(f"Successfully removed delivery with ID {idEntry} to database")

    def getPickup(self, idEntry) -> (int, int):
        query = f"""SELECT pickupX, pickupY
                    FROM deliveries 
                    WHERE id = {idEntry}"""

        return self.getCoordinatesResult(query)

    def getDropoff(self, idEntry) -> (int, int):
        query = f"""SELECT pickupX, pickupY
                    FROM deliveries 
                    WHERE id = {idEntry}"""

        return self.getCoordinatesResult(query)


    def getCoordinatesResult(self, query) -> (int, int):
        result = self.cursor.execute(query)
        rows = result.fetchone()

        if rows is None or len(rows) == 0:
            raise ValueError(f"No deliveries found")

        return rows[0]


    def close(self):
        self.cursor.close()
        self.conn.close()


if __name__ == '__main__':
    # Testing purposes
    db = Database()
    db.getPickup(1)
    try:
        (x, y) = db.getPickup(2)
    except ValueError as e:
        print(e)
    db.close()

