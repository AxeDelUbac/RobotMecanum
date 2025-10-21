#include "commandProcessing.h"
#include <Arduino.h>

int i16DirectionXAxis;
int i16DirectionYAxis;
int iSpeedSetting;
int iSetpointInRpm;

/**
 * @brief Initialise l'état du module de traitement des commandes.
 *
 * Met les axes X/Y et le réglage de vitesse à leurs valeurs par défaut.
 */
void CommandProcessing_init(void)
{
    i16DirectionXAxis = 0;
    i16DirectionYAxis = 0;
    iSpeedSetting = 0;
    iSetpointInRpm = 0;
}

/**
 * @brief Change le palier de réglage de vitesse de manière cyclique.
 *
 * @param bSwitchSpeed true pour incrémenter le palier, false pour décrémenter.
 *
 * Le palier est borné entre 0 et 5 (comportement original).
 */
void CommandProcessing_switchSpeedSetting(bool bSwitchSpeed) {
    if(bSwitchSpeed == true)
    {
        iSpeedSetting++;
        if(iSpeedSetting > 8)
        {
            iSpeedSetting = 0;
        }
    }
    else if (bSwitchSpeed == false)
    {
        iSpeedSetting--;
        if(iSpeedSetting < 0)
        {
            iSpeedSetting = 5;
        }
    }
}

/**
 * @brief Calcule la consigne de vitesse en tr/min (RPM) à partir du palier courant.
 *
 * Utilise les variables btnA/btnB pour basculer le palier si nécessaire.
 *
 * @return int consigne de vitesse en RPM (iSpeedSetting * 50)
 */

char cBluetoothCommand;
float CommandProcessing_modifySetpointInRpm(void)
{
    cBluetoothCommand = BluetoothReception_retreiveData();
    if(cBluetoothCommand == 'F')
    {
        CommandProcessing_switchSpeedSetting(1);
    }
    else if (cBluetoothCommand == 'B')
    {
        CommandProcessing_switchSpeedSetting(0);
    }
    iSetpointInRpm = iSpeedSetting * 50;

    return iSetpointInRpm;
}

/**
 * @brief Détermine la direction du robot à partir des axes X et Y courants.
 *
 * Normalise i16DirectionXAxis et i16DirectionYAxis en {-1,0,1} puis retourne
 * une constante CMD_DIR_* correspondant à la direction (avant, arrière, diagonales...).
 *
 * @return int une valeur de l'énumération command_direction_e.
 */
int CommandProcessing_manageDirection(void)
{

    int normaliseX = (i16DirectionXAxis > 0) ? 1 : ((i16DirectionXAxis < 0) ? -1 : 0);
    int normaliseY = (i16DirectionYAxis > 0) ? 1 : ((i16DirectionYAxis < 0) ? -1 : 0);

    if (normaliseX == 0 && normaliseY == 0) return CMD_DIR_STOP;
    else if (normaliseX == 0 && normaliseY == 1) return CMD_DIR_FRONT;
    else if (normaliseX == 0 && normaliseY == -1) return CMD_DIR_BACK;
    else if (normaliseX == -1 && normaliseY == 0) return CMD_DIR_LEFT;
    else if (normaliseX == 1 && normaliseY == 0) return CMD_DIR_RIGHT;
    else if (normaliseX == -1 && normaliseY == 1) return CMD_DIR_FRONT_LEFT;
    else if (normaliseX == 1 && normaliseY == 1) return CMD_DIR_FRONT_RIGHT;
    else if (normaliseX == -1 && normaliseY == -1) return CMD_DIR_BACK_LEFT;
    else if (normaliseX == 1 && normaliseY == -1) return CMD_DIR_BACK_RIGHT;

    return CMD_DIR_STOP;
}

/**
 * @brief Met à jour les valeurs X/Y internes utilisées pour le calcul de direction.
 *
 * @param iDirectionX valeur brute de l'axe X (sera normalisée lors du calcul)
 * @param iDirectionY valeur brute de l'axe Y (sera normalisée lors du calcul)
 */
void CommandProcessing_manageDirection(int iDirectionX, int iDirectionY){
    i16DirectionXAxis = iDirectionX;
    i16DirectionYAxis = iDirectionY;
}