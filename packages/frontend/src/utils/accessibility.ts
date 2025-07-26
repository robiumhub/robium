// Accessibility utilities and testing functions

/**
 * Calculate color contrast ratio between two colors
 * @param color1 - First color (hex, rgb, or rgba)
 * @param color2 - Second color (hex, rgb, or rgba)
 * @returns Contrast ratio
 */
export const calculateContrastRatio = (
  color1: string,
  color2: string
): number => {
  const getLuminance = (color: string): number => {
    // Convert color to RGB
    let r: number, g: number, b: number;

    if (color.startsWith('#')) {
      const hex = color.slice(1);
      r = parseInt(hex.slice(0, 2), 16);
      g = parseInt(hex.slice(2, 4), 16);
      b = parseInt(hex.slice(4, 6), 16);
    } else if (color.startsWith('rgb')) {
      const match = color.match(
        /rgba?\((\d+),\s*(\d+),\s*(\d+)(?:,\s*[\d.]+)?\)/
      );
      if (!match) return 0;
      [, r, g, b] = match.map(Number);
    } else {
      return 0;
    }

    // Convert to sRGB
    const [rs, gs, bs] = [r, g, b].map((c) => {
      c = c / 255;
      return c <= 0.03928 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
    });

    return 0.2126 * rs + 0.7152 * gs + 0.0722 * bs;
  };

  const l1 = getLuminance(color1);
  const l2 = getLuminance(color2);

  const lighter = Math.max(l1, l2);
  const darker = Math.min(l1, l2);

  return (lighter + 0.05) / (darker + 0.05);
};

/**
 * Check if contrast ratio meets WCAG AA standards
 * @param contrastRatio - Calculated contrast ratio
 * @param isLargeText - Whether the text is large (18pt+ or 14pt+ bold)
 * @returns Object with pass status and level
 */
export const checkWCAGContrast = (
  contrastRatio: number,
  isLargeText: boolean = false
) => {
  const aaNormal = 4.5;
  const aaLarge = 3.0;
  const aaaNormal = 7.0;
  const aaaLarge = 4.5;

  const threshold = isLargeText ? aaLarge : aaNormal;
  const aaaThreshold = isLargeText ? aaaLarge : aaaNormal;

  return {
    passes: contrastRatio >= threshold,
    level:
      contrastRatio >= aaaThreshold
        ? 'AAA'
        : contrastRatio >= threshold
          ? 'AA'
          : 'Fail',
    ratio: contrastRatio,
    required: threshold,
  };
};

/**
 * Validate ARIA attributes
 * @param element - HTML element to validate
 * @returns Array of validation errors
 */
export const validateARIA = (element: HTMLElement): string[] => {
  const errors: string[] = [];

  // Check for required ARIA attributes
  const hasRole = element.hasAttribute('role');
  const hasAriaLabel = element.hasAttribute('aria-label');
  const hasAriaLabelledby = element.hasAttribute('aria-labelledby');
  const hasAriaDescribedby = element.hasAttribute('aria-describedby');

  // Check for invalid ARIA combinations
  if (hasAriaLabel && hasAriaLabelledby) {
    errors.push('Element has both aria-label and aria-labelledby attributes');
  }

  // Check for required attributes based on role
  const role = element.getAttribute('role');
  if (role === 'button' && !hasAriaLabel && !hasAriaLabelledby) {
    errors.push('Button role requires aria-label or aria-labelledby');
  }

  if (role === 'link' && !hasAriaLabel && !hasAriaLabelledby) {
    errors.push('Link role requires aria-label or aria-labelledby');
  }

  // Check for invalid aria-invalid values
  const ariaInvalid = element.getAttribute('aria-invalid');
  if (
    ariaInvalid &&
    !['true', 'false', 'grammar', 'spelling'].includes(ariaInvalid)
  ) {
    errors.push('Invalid aria-invalid value');
  }

  return errors;
};

/**
 * Check if element is keyboard accessible
 * @param element - HTML element to check
 * @returns Object with accessibility status
 */
export const checkKeyboardAccessibility = (element: HTMLElement) => {
  const isFocusable =
    element.tabIndex >= 0 ||
    element.tagName === 'BUTTON' ||
    element.tagName === 'A' ||
    element.tagName === 'INPUT' ||
    element.tagName === 'SELECT' ||
    element.tagName === 'TEXTAREA';

  const hasKeyboardHandler =
    element.onkeydown !== null ||
    element.onkeypress !== null ||
    element.onkeyup !== null;

  return {
    isFocusable,
    hasKeyboardHandler,
    tabIndex: element.tabIndex,
    accessible: isFocusable || hasKeyboardHandler,
  };
};

/**
 * Generate accessibility report for a component
 * @param componentName - Name of the component
 * @param element - Root element of the component
 * @returns Accessibility report object
 */
export const generateAccessibilityReport = (
  componentName: string,
  element: HTMLElement
) => {
  const ariaErrors = validateARIA(element);
  const keyboardAccess = checkKeyboardAccessibility(element);

  // Check for semantic HTML
  const semanticElements = element.querySelectorAll(
    'main, nav, header, footer, section, article, aside'
  );
  const hasSemanticStructure = semanticElements.length > 0;

  // Check for heading hierarchy
  const headings = element.querySelectorAll('h1, h2, h3, h4, h5, h6');
  const headingLevels = Array.from(headings).map((h) =>
    parseInt(h.tagName.charAt(1))
  );
  const hasProperHeadingHierarchy = headingLevels.every(
    (level, index) => index === 0 || level <= headingLevels[index - 1] + 1
  );

  // Check for alt text on images
  const images = element.querySelectorAll('img');
  const imagesWithAlt = Array.from(images).filter((img) =>
    img.hasAttribute('alt')
  );
  const allImagesHaveAlt = imagesWithAlt.length === images.length;

  return {
    componentName,
    timestamp: new Date().toISOString(),
    aria: {
      errors: ariaErrors,
      valid: ariaErrors.length === 0,
    },
    keyboard: keyboardAccess,
    semantic: {
      hasSemanticStructure,
      semanticElementCount: semanticElements.length,
    },
    headings: {
      hasProperHierarchy: hasProperHeadingHierarchy,
      headingCount: headings.length,
      levels: headingLevels,
    },
    images: {
      allHaveAlt: allImagesHaveAlt,
      totalImages: images.length,
      imagesWithAlt: imagesWithAlt.length,
    },
    overall: {
      score: calculateAccessibilityScore({
        aria: ariaErrors.length === 0,
        keyboard: keyboardAccess.accessible,
        semantic: hasSemanticStructure,
        headings: hasProperHeadingHierarchy,
        images: allImagesHaveAlt,
      }),
    },
  };
};

/**
 * Calculate overall accessibility score
 * @param checks - Object with boolean accessibility checks
 * @returns Score from 0 to 100
 */
const calculateAccessibilityScore = (
  checks: Record<string, boolean>
): number => {
  const totalChecks = Object.keys(checks).length;
  const passedChecks = Object.values(checks).filter(Boolean).length;
  return Math.round((passedChecks / totalChecks) * 100);
};

/**
 * Focus management utilities
 */
export const focusManagement = {
  /**
   * Trap focus within a container
   */
  trapFocus: (container: HTMLElement) => {
    const focusableElements = container.querySelectorAll(
      'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
    );

    const firstElement = focusableElements[0] as HTMLElement;
    const lastElement = focusableElements[
      focusableElements.length - 1
    ] as HTMLElement;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Tab') {
        if (e.shiftKey) {
          if (document.activeElement === firstElement) {
            e.preventDefault();
            lastElement.focus();
          }
        } else {
          if (document.activeElement === lastElement) {
            e.preventDefault();
            firstElement.focus();
          }
        }
      }
    };

    container.addEventListener('keydown', handleKeyDown);
    return () => container.removeEventListener('keydown', handleKeyDown);
  },

  /**
   * Return focus to previous element
   */
  returnFocus: (element: HTMLElement) => {
    const returnElement = document.activeElement as HTMLElement;
    element.focus();
    return returnElement;
  },

  /**
   * Focus first focusable element in container
   */
  focusFirst: (container: HTMLElement) => {
    const focusableElement = container.querySelector(
      'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
    ) as HTMLElement;

    if (focusableElement) {
      focusableElement.focus();
    }
  },
};

export default {
  calculateContrastRatio,
  checkWCAGContrast,
  validateARIA,
  checkKeyboardAccessibility,
  generateAccessibilityReport,
  focusManagement,
};
