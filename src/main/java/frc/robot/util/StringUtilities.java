package frc.robot.util;

/**
 * A collection of utility methods for working with strings.
 */
public class StringUtilities {
	
	/**
	 * Returns a new version of the input string with the first letter
	 * capitalized.
	 *
	 * @param string The string to capitalize.
	 * @return A new version of the input string with the first letter
	 * capitalized.
	 */
	public static String capitalize(String string) {
		
		return string.substring(0, 1).toUpperCase() + string.substring(1);
		
	}
	
	/**
	 * Returns a new version of the input string with the first letter
	 * of each word capitalized.
	 *
	 * @param string The string to title case.
	 * @return A new version of the input string with the first letter
	 * of each word capitalized.
	 */
	public static String toTitleCase(String string) {
		
		String[] words = string.toLowerCase().split(" ");
		
		for (int i = 0; i < words.length; i++) {
			
			words[i] = words[i].toUpperCase();
			
		}
		
		return String.join(" ", words);
		
	}
	
}
