#!/usr/bin/env bash
# Source this file in every terminal that must connect to official practice.

export GZ_PARTITION="${GZ_PARTITION:-urrc_practice_$(id -u)}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

_team01_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_team01_search="$_team01_dir"
_team01_models=""
while [[ "$_team01_search" != "/" ]]; do
  if [[ -d "$_team01_search/official_urrc/src/urrc_track_gazebo/models" ]]; then
    _team01_models="$_team01_search/official_urrc/src/urrc_track_gazebo/models"
    break
  fi
  _team01_search="$(dirname -- "$_team01_search")"
done

if [[ -n "$_team01_models" ]]; then
  case ":${GZ_SIM_RESOURCE_PATH:-}:" in
    *":$_team01_models:"*) ;;
    *) export GZ_SIM_RESOURCE_PATH="$_team01_models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" ;;
  esac
fi

echo "Team 01 practice environment:"
echo "  GZ_PARTITION=$GZ_PARTITION"
echo "  GZ_IP=$GZ_IP"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<inherited/default>}"
echo "  world=monza"

unset _team01_dir _team01_search _team01_models
