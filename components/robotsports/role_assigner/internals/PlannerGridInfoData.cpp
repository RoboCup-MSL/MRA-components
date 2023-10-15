/*
 * TeamPlannerGrid.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jurge
 */
#include <sstream>
#include <iostream>
#include <fstream>
#include "PlannerGridInfoData.h"

using namespace trs;
using namespace std;

string PlannerGridInfoData::toString() {
	std::stringstream buffer;
	buffer << "TEAM:" << endl;
	for (auto it = this->gameData.Team.begin(); it != this->gameData.Team.end(); ++it) {
		buffer << "x: "<< it->getPosition().getVector2D().m_x << " y: " <<  it->getPosition().getVector2D().m_y << endl;
	}
	buffer << "OPPONENTS:" << endl;
	for (auto it = this->gameData.Opponents.begin(); it != this->gameData.Opponents.end(); ++it) {
		buffer << "x: "<< it->getPosition().getVector2D().m_x << " y: " <<  it->getPosition().getVector2D().m_y << endl;
	}
	buffer<< "BALL x: "  <<  this->gameData.ball.getPosition().getVector2D().m_x << " y: "<< this->gameData.ball.getPosition().getVector2D().m_y << endl;
	buffer << "LAYER-NAMES:" << endl;
	for (auto it = this->name.begin();  it != this->name.end(); ++it) {
		buffer <<  *it << endl;
	}
	buffer << "LAYER-WEIGHTS:" << endl;
	for (auto it = this->weight.begin();  it != this->weight.end(); ++it) {
		buffer << *it << endl;
	}
	return buffer.str();
}

void PlannerGridInfoData::saveToFile(const std::string& filename) {
	FILE* fp = fopen(filename.c_str(), "w");
	fprintf(fp, "# GAME-DATA\n");
	// TEAM data
	for (auto it = this->gameData.Team.begin(); it != this->gameData.Team.end(); ++it) {
		fprintf(fp, " %5.2f; %5.2f;", it->getPosition().getVector2D().m_x, it->getPosition().getVector2D().m_y);
	}
	fprintf(fp, "\n");
	for (auto it = this->gameData.Opponents.begin(); it != this->gameData.Opponents.end(); ++it) {
		fprintf(fp, " %5.2f; %5.2f;", it->getPosition().getVector2D().m_x, it->getPosition().getVector2D().m_y);
	}
	fprintf(fp, "\n");
	fprintf(fp, " %5.2f; %5.2f;\n", this->gameData.ball.getPosition().getVector2D().m_x, this->gameData.ball.getPosition().getVector2D().m_y);
	fprintf(fp, "# LAYER-NAMES\n");
	for (auto it = this->name.begin();  it != this->name.end(); ++it) {
		fprintf(fp, "%s;", it->c_str());
	}
	fprintf(fp, "\n");

	fprintf(fp, "# LAYER-WEIGHTS\n");
	for (auto it = this->weight.begin();  it != this->weight.end(); ++it) {
		fprintf(fp, "%8.6e;", *it);
	}
	fprintf(fp, "\n");

	fprintf(fp, "# CELL-DATA\n");
	for (auto it = this->cells.begin();  it != this->cells.end(); ++it) {
		fprintf(fp, "%d;%5.2f;%5.2f;", it->id, it->x, it->y);
		for (auto val_it = it->value.begin();  val_it != it->value.end(); ++val_it) {
			fprintf(fp, "%8.6e;", *val_it);
		}
		fprintf(fp, "\n");  // currently empty: to be filled in.
	}
	fclose(fp);
}

void PlannerGridInfoData::readFromFile(const std::string& filename) {
	std::string sLine = "";
	std::ifstream infile;

	infile.open(filename.c_str());
//	logAlways("READING file: %s", filename.c_str());

	bool gameData_comment = true;
	bool gameData_data_team = false;
	bool gameData_data_opponents = false;
	bool gameData_data_ball = false;
	bool layer_names_comment = false;
	bool layer_names_data = false;
	bool layer_weights_comment = false;
	bool layer_weights_data = false;
	bool cell_comment = false;
	bool cell_data = false;
	std::string delimiter = ";";
	while (!infile.eof())
	{
		getline(infile, sLine);
		if (gameData_comment) {
			gameData_comment = false;
			gameData_data_team = true;
		}
		else if (gameData_data_team) {
			std::string item_text;
			unsigned item_nr = 0;
			size_t pos = 0;
			double x, y;
			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				item_nr++;
				item_text = sLine.substr(0, pos);
				sLine.erase(0, pos + delimiter.length());
				if (item_nr == 1) {
					x = std::stod(item_text);
				}
				if (item_nr == 2) {
					y = std::stod(item_text);
				}
				if (item_nr == 2) {
					this->gameData.Team.push_back(MovingObject(x, y, 0.0, 0.0, 0.0, 0.0, 0, true));
					item_nr = 0;
				}
			}

			gameData_data_team = false;
			gameData_data_opponents = true;
		}
		else if (gameData_data_opponents) {
			std::string item_text;
			unsigned item_nr = 0;
			size_t pos = 0;
			double x, y;
			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				item_nr++;
				item_text = sLine.substr(0, pos);
				sLine.erase(0, pos + delimiter.length());
				if (item_nr == 1) {
					x = std::stod(item_text);
				}
				if (item_nr == 2) {
					y = std::stod(item_text);
				}
				if (item_nr == 2) {
					this->gameData.Opponents.push_back(MovingObject(x, y, 0.0, 0.0, 0.0, 0.0, 0, true));
					item_nr = 0;
				}
			}

			gameData_data_opponents = false;
			gameData_data_ball = true;
		}
		else if (gameData_data_ball) {
			gameData_data_ball = false;
			std::string item_text;
			unsigned item_nr = 0;
			size_t pos = 0;
			double x, y;
			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				item_nr++;
				item_text = sLine.substr(0, pos);
				sLine.erase(0, pos + delimiter.length());
				if (item_nr == 1) {
					x = std::stod(item_text);
				}
				if (item_nr == 2) {
					y = std::stod(item_text);
				}
			}
			if (item_nr == 2) {
				this->gameData.ball = MovingObject(x, y, 0.0, 0.0, 0.0, 0.0, 0, true);
			}
			layer_names_comment = true;
		}
		else if (layer_names_comment) {
			layer_names_comment = false;
			layer_names_data = true;
		}
		else if (layer_names_data) {
			size_t pos = 0;
			std::string layer_name;
			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				layer_name = sLine.substr(0, pos);
				name.push_back(layer_name);
				sLine.erase(0, pos + delimiter.length());
			}
			layer_names_data = false;
			layer_weights_comment = true;
		}
		else if (layer_weights_comment) {
			layer_weights_comment = false;
			layer_weights_data = true;
		}
		else if (layer_weights_data) {
			size_t pos = 0;
			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				std::string layer_weight_string = sLine.substr(0, pos);
				weight.push_back(std::stod(layer_weight_string));
				sLine.erase(0, pos + delimiter.length());
			}
			layer_weights_data = false;
			cell_comment = true;
		}
		else if (cell_comment) {
			cell_comment = false;
			cell_data = true;
		}
		else if (cell_data) {
			int item_nr = 0;
			size_t pos = 0;
			std::string item_text;
			unsigned id;
			double x;
			double y;
			std::vector<double> values;

			while ((pos = sLine.find(delimiter)) != std::string::npos) {
				item_nr++;
				item_text = sLine.substr(0, pos);
				sLine.erase(0, pos + delimiter.length());
				if (item_nr == 1) {
					id = std::stoi(item_text);
				}
				if (item_nr == 2) {
					x = std::stod(item_text);
				}
				if (item_nr == 3) {
					y = std::stod(item_text);
				}
				if (item_nr > 3) {
					double v = std::stod(item_text);
					values.push_back(v);
				}
			}
			if (item_nr > 0) {
				this->cells.push_back(PlannerGridCell(id, x, y, values));
			}
			else {
				break;
			}
		}
	}

	infile.close();
//	logAlways("Read file completed!!");
}
