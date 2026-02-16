/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of ivS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * ivS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef BIMDATA_H
#define BIMDATA_H

#include <vector>
#include <mutex>
#include "Geometric/Plane.h"
// #include "Room.h"
// #include "Column.h"
// ...add other BIM element headers as needed

namespace ORB_SLAM3 {

class BIMDatabase {
public:
    // Walls
    void AddWallBIM(Plane* wall) { 
        std::unique_lock<std::mutex> lock(mMutexBIMDatabase);
        mvWallsBIM.push_back(wall); 
    }
    std::vector<Plane*> GetWallsBIM() const { 
        unique_lock<std::mutex> lock(mMutexBIMDatabase);
        return mvWallsBIM; 
    }

    void SetWallsBIM(const std::vector<Plane*>& walls) { 
        std::unique_lock<std::mutex> lock(mMutexBIMDatabase);
        mvWallsBIM.clear();  // Remove all existing walls
        mvWallsBIM = walls;  // Set new walls
    }

    void ClearWallsBIM() { 
        std::unique_lock<std::mutex> lock(mMutexBIMDatabase);
        mvWallsBIM.clear(); 
    }

    // // Rooms
    // void AddRoomBIM(Room* room) { 
    //     std::unique_lock<std::mutex> lock(mMutexBIMDatabase);
    //     mvRooms.push_back(room); 
    // }
    // std::vector<Room*> GetRoomsBIM() const { 
    //     std::unique_lock<std::mutex> lock(mMutexBIMDatabase);
    //     return mvRooms; 
    // }

    // ...add similar for corridors, doors, etc.

private:
    std::vector<Plane*> mvWallsBIM;
    // std::vector<Room*> mvRooms;
    // std::vector<Column*> mvColumns;
    // ...add other BIM element containers

    // Mutex for thread safety
    mutable std::mutex mMutexBIMDatabase;
};

}

#endif