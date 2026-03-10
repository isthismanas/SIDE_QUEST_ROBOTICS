using System.Collections;
using UnityEngine;

public class UIAudioPlayer : MonoBehaviour
{
    [Header("Core")]
    [SerializeField] private AudioSource audioSource;

    [Header("UI Clicks")]
    [SerializeField] private AudioClip primaryClick;     // Start, Drop
    [SerializeField] private AudioClip secondaryClick;   // Fix, Nudge
    [SerializeField] private AudioClip hoverTick;        // optional

    [Header("Gameplay")]
    [SerializeField] private AudioClip runSuccess;
    [SerializeField] private AudioClip comboBoost;
    [SerializeField] private AudioClip tumble;
    [SerializeField] private AudioClip greenDrop;
    [SerializeField] private AudioClip yellowDrop;
    [SerializeField] private AudioClip redDrop;
    [SerializeField] private AudioClip boostEnd;

    [Header("Tuning")]
    [SerializeField] private float hoverVolume = 0.85f;
    [SerializeField] private float gameplayVolume = 1.0f;
    [SerializeField] private float greenDropDelay = 2.0f;

    private Coroutine greenDropRoutine;

    public void PlayPrimary()
    {
        PlayClip(primaryClick);
    }

    public void PlaySecondary()
    {
        PlayClip(secondaryClick);
    }

    public void PlayHover()
    {
        PlayClip(hoverTick, hoverVolume);
    }

    public void PlayRunSuccess()
    {
        PlayClip(runSuccess, gameplayVolume);
    }

    public void PlayComboBoost()
    {
        PlayClip(comboBoost, gameplayVolume);
    }

    public void PlayTumble()
    {
        PlayClip(tumble, gameplayVolume);
    }

    public void PlayBoostEnd()
    {
        PlayClip(boostEnd, gameplayVolume);
    }

    public void PlayGreenDrop()
    {
        PlayClip(greenDrop, gameplayVolume);
    }

    public void PlayGreenDropDelayed()
    {
        PlayDropDelayed("GREEN");
    }

    public void PlayDropDelayed(string zone)
    {
        if (!isActiveAndEnabled)
        {
            PlayClip(ResolveDropClip(zone), gameplayVolume);
            return;
        }

        if (greenDropRoutine != null)
            StopCoroutine(greenDropRoutine);

        greenDropRoutine = StartCoroutine(PlayDropAfterDelay(zone));
    }

    private IEnumerator PlayDropAfterDelay(string zone)
    {
        yield return new WaitForSeconds(greenDropDelay);
        PlayClip(ResolveDropClip(zone), gameplayVolume);
        greenDropRoutine = null;
    }

    private AudioClip ResolveDropClip(string zone)
    {
        string normalized = string.IsNullOrWhiteSpace(zone) ? "GREEN" : zone.Trim().ToUpperInvariant();

        if (normalized == "YELLOW")
            return yellowDrop != null ? yellowDrop : greenDrop;

        if (normalized == "RED")
            return redDrop != null ? redDrop : greenDrop;

        return greenDrop;
    }

    private void PlayClip(AudioClip clip, float volume = 1.0f)
    {
        if (audioSource == null || clip == null)
            return;

        audioSource.PlayOneShot(clip, volume);
    }
}