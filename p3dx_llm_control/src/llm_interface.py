#!/usr/bin/env python3

import google.generativeai as genai
import json
import os
from typing import Dict, Optional

class LLMInterface:
    """Interface for Google Gemini Flash LLM to parse robot motion commands."""

    def __init__(self, api_key: str, model_name: str = "gemini-2.5-flash",
                 prompt_file: str = None, timeout: float = 5.0):
        """
        Initialize the LLM interface.

        Args:
            api_key: Google Gemini API key
            model_name: Model name to use (default: gemini-1.5-flash)
            prompt_file: Path to system prompt file
            timeout: API timeout in seconds
        """
        if not api_key:
            raise ValueError("API key is required")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(model_name)
        self.timeout = timeout
        self.system_prompt = self._load_system_prompt(prompt_file)
        self.request_count = 0

    def _load_system_prompt(self, prompt_file: Optional[str]) -> str:
        """Load system prompt from file or use default."""
        if prompt_file and os.path.exists(prompt_file):
            with open(prompt_file, 'r') as f:
                return f.read()
        else:
            # Default system prompt (fallback - matches velocity_limits.yaml)
            return """You are a robot motion controller. Parse user commands into JSON format.
Output ONLY valid JSON, no other text.
Format: {"action": "move_forward"|"move_backward"|"turn_left"|"turn_right"|"stop",
         "linear_velocity": <float in m/s, max 0.25>,
         "angular_velocity": <float in rad/s, max 0.5>,
         "duration": <float in seconds, max 15.0>}"""

    def parse_command(self, user_text: str) -> Dict:
        """
        Parse user command using LLM and return structured motion command.

        Args:
            user_text: User's natural language command

        Returns:
            Dictionary with keys: action, linear_velocity, angular_velocity, duration

        Raises:
            ValueError: If LLM response is invalid
            RuntimeError: If API call fails
        """
        try:
            # Increment request counter
            self.request_count += 1

            # Combine system prompt and user command
            full_prompt = f"{self.system_prompt}\n\nUser command: {user_text}\n\nRespond with JSON only:"

            # Call Gemini API
            response = self.model.generate_content(full_prompt)

            # Extract text from response
            response_text = response.text.strip()

            # Try to extract JSON from response (in case LLM adds extra text)
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1

            if json_start == -1 or json_end == 0:
                raise ValueError(f"No JSON found in response: {response_text}")

            json_text = response_text[json_start:json_end]

            # Parse JSON
            command = json.loads(json_text)

            # Validate response structure
            if not self.validate_response(command):
                raise ValueError(f"Invalid command structure: {command}")

            return command

        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse JSON from LLM response: {e}")
        except Exception as e:
            raise RuntimeError(f"LLM API call failed: {e}")

    def validate_response(self, response: Dict) -> bool:
        """
        Validate LLM response structure and values.

        Args:
            response: Parsed JSON response

        Returns:
            True if valid, False otherwise
        """
        required_keys = ['action', 'linear_velocity', 'angular_velocity', 'duration']

        # Check all required keys exist
        if not all(key in response for key in required_keys):
            return False

        # Validate action
        valid_actions = ['move_forward', 'move_backward', 'turn_left', 'turn_right', 'stop']
        if response['action'] not in valid_actions:
            return False

        # Validate numeric types
        try:
            linear_vel = float(response['linear_velocity'])
            angular_vel = float(response['angular_velocity'])
            duration = float(response['duration'])
        except (ValueError, TypeError):
            return False

        # Validate ranges (allow some tolerance)
        if abs(linear_vel) > 0.6:  # Slightly higher than max for tolerance
            return False
        if abs(angular_vel) > 0.6:  # Increased from 1.1 to match 0.5 max + tolerance
            return False
        if duration < 0 or duration > 16.0:  # Increased from 11.0 to 16.0 to allow 15s + tolerance
            return False

        return True

    def get_request_count(self) -> int:
        """Get total number of API requests made."""
        return self.request_count


# Simple test function
if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python3 llm_interface.py <API_KEY> [command]")
        sys.exit(1)

    api_key = sys.argv[1]
    command = " ".join(sys.argv[2:]) if len(sys.argv) > 2 else "move forward 2 meters"

    # Find the prompt file (works both in source and installed locations)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    prompt_file = os.path.join(script_dir, '..', 'prompts', 'system_prompt.txt')
    if not os.path.exists(prompt_file):
        # Try source location
        prompt_file = '/home/robot/vishwa/p3dx_ws/src/p3dx_llm_control/prompts/system_prompt.txt'
    
    llm = LLMInterface(api_key, prompt_file=prompt_file)

    try:
        result = llm.parse_command(command)
        print(f"Command: {command}")
        print(f"Parsed: {json.dumps(result, indent=2)}")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
